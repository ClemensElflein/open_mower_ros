package cli

import (
	"fmt"
	"path/filepath"

	"github.com/spf13/cobra"

	"github.com/ClemensElflein/open_mower_ros/utils/soundctl/internal/sync"
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
		RunE: func(cmd *cobra.Command, args []string) error {
			cfg, err := sync.LoadConfig(config)
			if err != nil {
				return err
			}
			ids, err := sync.LoadSoundIDs(soundIDs)
			if err != nil {
				return err
			}

			errs := cfg.Validate(ids)
			for _, e := range errs {
				fmt.Fprintln(cmd.ErrOrStderr(), "  [error]", e)
			}

			if checkMP3 {
				for _, file := range cfg.Files() {
					info, err := sync.CheckMP3(filepath.Join(cfg.SoundPath, file))
					if err != nil {
						e := fmt.Sprintf("%s: %v", file, err)
						fmt.Fprintln(cmd.ErrOrStderr(), "  [error]", e)
						errs = append(errs, e)
						continue
					}
					if !info.Is16kMono() {
						e := fmt.Sprintf("%s: not 16 kHz mono (rate=%d channels=%d)", file, info.SampleRate, info.Channels)
						fmt.Fprintln(cmd.ErrOrStderr(), "  [error]", e)
						errs = append(errs, e)
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
	cmd.Flags().StringVar(&soundIDs, "sound-ids", defaultSoundIDsPath, "path to sound_service.json")
	cmd.Flags().BoolVar(&checkMP3, "check-mp3", false, "verify MP3s are 16 kHz mono")
	return cmd
}
