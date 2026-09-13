package cli

import (
	"context"
	"fmt"
	"os"
	"os/signal"
	"strconv"
	"syscall"
	"time"

	"github.com/spf13/cobra"

	"github.com/ClemensElflein/open_mower_ros/utils/soundctl/internal/fwsync"
)

// playFlags are the connection flags shared by all play/stop subcommands.
type playFlags struct {
	soundIDs      string
	bind          string
	addr          string
	masterVolume  int
	discoveryWait time.Duration
}

func (f *playFlags) add(cmd *cobra.Command) {
	cmd.PersistentFlags().StringVar(&f.soundIDs, "sound-ids", defaultSoundIDsPath, "path to the shared definition (sound_service.json)")
	cmd.PersistentFlags().StringVar(&f.bind, "bind", "0.0.0.0", "local IP to bind for discovery")
	cmd.PersistentFlags().StringVar(&f.addr, "addr", "", "SoundService \"ip:port\" (skips the slow discovery)")
	cmd.PersistentFlags().DurationVar(&f.discoveryWait, "wait", 20*time.Second, "how long to wait for the SoundService")
	cmd.PersistentFlags().IntVar(&f.masterVolume, "master-volume", -1, "master volume 0..100 to set first (default: unchanged)")
}

func (f *playFlags) options() fwsync.PlayOptions {
	return fwsync.PlayOptions{
		SoundIDsPath: f.soundIDs,
		BindIP:       f.bind,
		Addr:         f.addr,
		Volume:       f.masterVolume,
	}
}

func (f *playFlags) run(opts fwsync.PlayOptions) error {
	ctx, stop := signal.NotifyContext(context.Background(), os.Interrupt, syscall.SIGTERM)
	defer stop()
	ctx, cancel := context.WithTimeout(ctx, f.discoveryWait)
	defer cancel()
	return fwsync.Play(ctx, opts)
}

// parseInt parses an integer argument. The value ranges are checked in the sync
// package (there they are shared with the YAML validation).
func parseInt(name, s string) (int, error) {
	v, err := strconv.Atoi(s)
	if err != nil {
		return 0, fmt.Errorf("%s: %q is not a number", name, s)
	}
	return v, nil
}

func newPlayCmd() *cobra.Command {
	f := &playFlags{}
	cmd := &cobra.Command{
		Use:   "play",
		Short: "Play a tone/sequence/MP3/sound on the firmware at runtime",
		Long: "Trigger playback on the SoundService without touching the persisted sound\n" +
			"definitions — handy for auditioning sounds on the robot.\n" +
			"\n" +
			"Values are checked with the same rules as the sounds_*.yaml definitions. The\n" +
			"RPCs are fire and forget (the firmware answers the service owner, i.e. the\n" +
			"high-level system), so the only feedback is the sound itself.\n" +
			"\n" +
			"Every call discovers the SoundService (a few seconds, because claimed services\n" +
			"advertise rarely). Pass --addr <ip:port> — the address is printed by\n" +
			"\"soundctl sync\" — to talk to it directly and instantly.\n" +
			"\n" +
			"tone/sequence/mp3 take --preempt: stop a running sound, clear the queue and play\n" +
			"immediately. Configured sounds ignore it — their definition decides (\"preempt\"\n" +
			"in sounds_*.yaml, set for EMERGENCY).\n" +
			"\n" +
			"Playback is stopped with \"soundctl stop\" (not \"soundctl play stop\").",
	}

	f.add(cmd)

	var volume int
	var wave string
	var unison, detune int
	var attack, decay int
	var preempt bool

	tone := &cobra.Command{
		Use:   "tone <freq> <duration_ms>",
		Short: "Play a single synthesised tone",
		Long: "Play one sine tone: <freq> in Hz and <duration_ms> in milliseconds.\n" +
			"Use \"play sequence\" for other waveforms, several notes or unison voices.",
		Example: "  soundctl play tone 400 60 --addr 172.16.78.150:49154",
		Args:    cobra.ExactArgs(2),
		RunE: func(cmd *cobra.Command, args []string) error {
			freq, err := parseInt("freq", args[0])
			if err != nil {
				return err
			}
			dur, err := parseInt("duration_ms", args[1])
			if err != nil {
				return err
			}
			opts := f.options()
			opts.Preempt = preempt
			opts.Tone = &fwsync.ToneSpec{Freq: freq, DurationMs: dur, Volume: volume}
			return f.run(opts)
		},
	}
	tone.Flags().IntVar(&volume, "volume", 80, "per-definition volume 0..100")
	tone.Flags().BoolVar(&preempt, "preempt", false, "stop a running sound, clear the queue and play now")

	sequence := &cobra.Command{
		Use:   `sequence "<freq:dur[:lfoHzx10[:lfoDepth]] [note2 ...]>"`,
		Short: "Play a compact note sequence, e.g. \"250:60 0:40 375:80\"",
		Long: "Play a sequence of synthesised notes, each given as\n" +
			"\n" +
			"  freq:dur[:lfoHzx10[:lfoDepth]]\n" +
			"\n" +
			"with freq in Hz (0 = pause), dur in ms and an optional vibrato (rate in 0.1 Hz\n" +
			"steps, then its depth in Hz). Space, comma, semicolon and tab separate notes;\n" +
			"up to 8 notes fit into 160 characters.\n" +
			"\n" +
			"--attack/--decay shape every note (0..255 ms): a linear fade-in and an\n" +
			"exponential fade to about -60 dB. A single note with a decay is a \"ping\"\n" +
			"instead of a hard-gated rectangle.",
		Example: "  soundctl play sequence \"250:60 0:40 375:80\" --wave sine --volume 45\n" +
			"  soundctl play sequence \"880:150:20:30\" --wave saw --unison 3 --detune 12\n" +
			"  soundctl play sequence \"554:360\" --wave saw --attack 4 --decay 200",
		Args: cobra.ExactArgs(1),
		RunE: func(cmd *cobra.Command, args []string) error {
			opts := f.options()
			opts.Preempt = preempt
			opts.Sequence = &fwsync.SequenceSpec{
				Notes:    args[0],
				Waveform: wave,
				Volume:   volume,
				Unison:   unison,
				DetuneHz: detune,
				AttackMs: attack,
				DecayMs:  decay,
			}
			return f.run(opts)
		},
	}
	sequence.Flags().IntVar(&volume, "volume", 80, "per-definition volume 0..100")
	sequence.Flags().StringVar(&wave, "wave", "sine", "oscillator waveform: sine|square|triangle|saw")
	sequence.Flags().IntVar(&unison, "unison", 1, "detuned voices (1 = single, odd: 3/5/7)")
	sequence.Flags().IntVar(&detune, "detune", 0, "frequency spread between unison voices in Hz")
	sequence.Flags().IntVar(&attack, "attack", 0, "per-note attack ramp in ms (0..255, 0 = instant)")
	sequence.Flags().IntVar(&decay, "decay", 0, "per-note fade to ~-60 dB in ms (0..255, 0 = hold the note)")
	sequence.Flags().BoolVar(&preempt, "preempt", false, "stop a running sound, clear the queue and play now")

	sound := &cobra.Command{
		Use:   "sound <name>",
		Short: "Play a configured sound by name, e.g. boot_ping",
		Long: "Play a sound of the shared SoundService definition by its name (see the keys\n" +
			"of \"sounds\" in sounds_*.yaml). Names are matched case-insensitively; unknown\n" +
			"names are reported with the list of valid ones.",
		Example: "  soundctl play sound boot_ping",
		Args:    cobra.ExactArgs(1),
		RunE: func(cmd *cobra.Command, args []string) error {
			opts := f.options()
			opts.Sound = args[0]
			return f.run(opts)
		},
	}

	mp3 := &cobra.Command{
		Use:   "mp3 <path>",
		Short: "Play an MP3 stored on the firmware (16 kHz mono)",
		Long: "Play an MP3 by its path on the firmware (usually /sounds/<file>, at most 128\n" +
			"characters). Files are uploaded or updated with \"soundctl sync\", which also\n" +
			"verifies that they are 16 kHz mono.",
		Example: "  soundctl play mp3 /sounds/en_hi-i-am-steve.mp3",
		Args:    cobra.ExactArgs(1),
		RunE: func(cmd *cobra.Command, args []string) error {
			opts := f.options()
			opts.Preempt = preempt
			opts.Mp3Path = args[0]
			return f.run(opts)
		},
	}
	mp3.Flags().BoolVar(&preempt, "preempt", false, "stop a running sound, clear the queue and play now")

	cmd.AddCommand(tone, sequence, sound, mp3)
	return cmd
}

func newStopCmd() *cobra.Command {
	f := &playFlags{}
	cmd := &cobra.Command{
		Use:   "stop",
		Short: "Stop playback and flush the firmware player queues",
		Long: "Stop playback and flush the firmware player queues. Unlike the other commands\n" +
			"this needs no sound definition, so it still works when sound_service.json is\n" +
			"not available.",
		Example: "  soundctl stop",
		Args:    cobra.NoArgs,
		RunE: func(cmd *cobra.Command, args []string) error {
			opts := f.options()
			opts.Stop = true
			return f.run(opts)
		},
	}
	f.add(cmd)
	return cmd
}
