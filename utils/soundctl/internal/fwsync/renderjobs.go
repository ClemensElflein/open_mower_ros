package fwsync

import (
	"encoding/json"
	"fmt"
	"time"
)

// RenderJob is one MP3 that `soundctl tts` has to produce.
type RenderJob struct {
	Sound   string // sound id in the YAML ("boot_complete")
	Variant int    // 1-based number within the sound
	Label   string // "boot_complete" or "boot_complete[2]", for logs and --check
	File    string // MP3 file name in /sounds/
	// Identity is the render cache key of this variant.
	Identity TTSIdentity
}

// RenderJobs lists the variants that need a render, in a stable order. A variant
// with a file but without a tts text is a ready-made recording: there is nothing
// to synthesise, so it is not listed.
func (c *Config) RenderJobs() []RenderJob {
	set := c.TTSSettings()
	var jobs []RenderJob
	for _, name := range c.Names() {
		def := c.Sounds[name]
		if def.Type != "mp3" {
			continue
		}
		for i, v := range def.Variants {
			if v.TTS == "" {
				continue
			}
			label := name
			if len(def.Variants) > 1 {
				label = fmt.Sprintf("%s[%d]", name, i+1)
			}
			jobs = append(jobs, RenderJob{
				Sound:   name,
				Variant: i + 1,
				Label:   label,
				File:    v.File,
				Identity: TTSIdentity{
					Engine:          set.Engine,
					Model:           set.Model,
					Voice:           set.Voice,
					Language:        c.Language,
					Text:            v.TTS,
					Bitrate:         set.Bitrate,
					LoudNorm:        set.LoudNorm,
					Stability:       set.Stability,
					SimilarityBoost: set.SimilarityBoost,
					Style:           set.Style,
					UseSpeakerBoost: set.UseSpeakerBoost,
				},
			})
		}
	}
	return jobs
}

// Blob marshals the sound overrides into the JSON blob that is sent to the
// SoundService ("Sound Definitions" register). Only the "sounds" map is sent;
// the HL-only top-level fields (version, language, sound_path, tts) are
// stripped, exactly like the Input-Configs path strips name/actions.
func (c *Config) Blob() ([]byte, error) {
	// every_ms is derived here (the firmware does not parse duration strings);
	// copy the map so a second Blob call starts from the YAML values again.
	sounds := make(map[string]SoundDef, len(c.Sounds))
	for name, def := range c.Sounds {
		if def.Every != "" {
			d, err := time.ParseDuration(def.Every)
			if err != nil {
				return nil, fmt.Errorf("%s: invalid 'every' %q: %w", name, def.Every, err)
			}
			if d <= 0 {
				return nil, fmt.Errorf("%s: 'every' must be positive (got %q)", name, def.Every)
			}
			def.EveryMs = int(d.Milliseconds())
		}
		// The firmware starts with "first" for every sound (its persisted index is zeroed),
		// so the documented default has to travel in the blob - otherwise a sound with
		// several takes would always play take 1.
		if def.Pick == "" {
			def.Pick = PickRandom
		}
		sounds[name] = def
	}
	return json.Marshal(struct {
		Sounds map[string]SoundDef `json:"sounds"`
	}{Sounds: sounds})
}
