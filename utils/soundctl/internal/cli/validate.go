package cli

import (
	"fmt"
	"path/filepath"

	"github.com/spf13/cobra"

	"github.com/ClemensElflein/open_mower_ros/utils/soundctl/internal/fwsync"
)

func newValidateCmd() *cobra.Command {
	var (
		config   string
		soundIDs string
		checkMP3 bool
	)
	cmd := &cobra.Command{
		Use:   "validate",
		Short: "Validate a sounds_*.yaml file (and optionally its MP3s)",
		Long: "Check sounds_*.yaml against the shared SoundService definition (sound ids,\n" +
			"types, waveforms, value ranges) and, with --check-mp3, verify that every\n" +
			"referenced MP3 is 16 kHz mono as the firmware requires. Exits non-zero on\n" +
			"errors, so it can gate a build or a deploy.",
		Example: "  soundctl validate --check-mp3",
		RunE: func(cmd *cobra.Command, args []string) error {
			cfg, err := fwsync.LoadConfig(config)
			if err != nil {
				return err
			}
			ids, err := fwsync.LoadSoundIDs(soundIDs)
			if err != nil {
				return err
			}

			var errs []string
			report := func(format string, args ...any) {
				e := fmt.Sprintf(format, args...)
				errs = append(errs, e)
				fmt.Fprintln(cmd.ErrOrStderr(), "  [error]", e)
			}

			for _, e := range cfg.Validate(ids) {
				report("%s", e)
			}

			if checkMP3 {
				for _, file := range cfg.Files() {
					info, err := fwsync.CheckMP3(filepath.Join(cfg.SoundPath, file))
					if err != nil {
						report("%s: %v", file, err)
						continue
					}
					if !info.Is16kMono() {
						report("%s: not 16 kHz mono (rate=%d channels=%d)", file, info.SampleRate, info.Channels)
					} else {
						fmt.Fprintf(cmd.OutOrStdout(), "  ok: %s (16 kHz mono)\n", file)
					}
				}
			}

			if len(errs) > 0 {
				return fmt.Errorf("%d error(s)", len(errs))
			}
			fmt.Fprintf(cmd.OutOrStdout(), "OK: %s\n", config)
			return nil
		},
	}
	cmd.Flags().StringVar(&config, "config", defaultConfigPath, "path to sounds_*.yaml")
	cmd.Flags().StringVar(&soundIDs, "sound-ids", defaultSoundIDsPath, "path to the shared definition (sound_service.json)")
	cmd.Flags().BoolVar(&checkMP3, "check-mp3", false, "verify MP3s are 16 kHz mono")
	return cmd
}
