package fwsync

import (
	"go.yaml.in/yaml/v3"
)

// TTSConfig is the top-level "tts:" block: the defaults for every sound with a
// tts variant.
//
// Engine-agnostic options are flat (bitrate, loudnorm), everything that belongs
// to an engine sits in its own block. Selecting an engine therefore stays a
// one-liner, and adding an engine is another block next to the existing ones.
//
//	bitrate  MP3 bitrate of the encoder (ffmpeg)
//	loudnorm EBU R128 normalisation in ffmpeg
type TTSConfig struct {
	Engine   string `yaml:"engine"`
	Bitrate  string `yaml:"bitrate"`
	LoudNorm *bool  `yaml:"loudnorm"`

	// ElevenLabs is used when engine is "elevenlabs" (the cloud engine).
	ElevenLabs ElevenLabsConfig `yaml:"elevenlabs"`

	// Cartesia is used when engine is "cartesia" (the other cloud engine).
	Cartesia CartesiaConfig `yaml:"cartesia"`

	// Unknown collects every key this block does not define. Without it a leftover
	// block from an older definition would be accepted silently and its texts would
	// be voiced by a different engine - the validator names the key instead (see
	// Validate).
	Unknown map[string]yaml.Node `yaml:",inline"`
}

// ElevenLabsConfig is the engine block of the ElevenLabs cloud engine.
type ElevenLabsConfig struct {
	// Model is the ElevenLabs model id ("" = DefaultTTSModel, i.e. eleven_v4).
	Model string `yaml:"model"`
	// Voice is the ElevenLabs voice *id* (not a name); `soundctl tts voices`
	// lists the ids of the account.
	Voice string `yaml:"voice"`
	// Stability, SimilarityBoost and Style are 0..1, UseSpeakerBoost is a
	// switch. They are sent with every request and are part of the render cache
	// key, so changing one re-renders the announcements.
	Stability       *float64 `yaml:"stability"`
	SimilarityBoost *float64 `yaml:"similarity_boost"`
	Style           *float64 `yaml:"style"`
	UseSpeakerBoost *bool    `yaml:"use_speaker_boost"`
}

// CartesiaConfig is used when tts.engine is "cartesia".
type CartesiaConfig struct {
	// Model is a Cartesia model id ("" = DefaultCartesiaModel, the sonic-3.6
	// release alias, i.e. the newest stable 3.6 build). A dated id pins one build.
	// Cartesia does not render bit-identically, so pinning only keeps the voice
	// character; the model is part of the render fingerprint, so changing the id
	// re-renders the whole set.
	Model string `yaml:"model"`
	// Voice is the Cartesia voice *id* (not a name);
	// `soundctl tts voices --engine cartesia` lists the ids of the account.
	Voice string `yaml:"voice"`
}

// TTSSettings is the resolved (i.e. defaulted) tts block. The resolved values are
// part of the render cache key, so a change in the YAML or in the defaults above
// invalidates the existing renders.
type TTSSettings struct {
	Engine          string
	Bitrate         string
	LoudNorm        bool
	Model           string // engine model id (ElevenLabs or Cartesia)
	Voice           string // engine voice id
	Stability       float64
	SimilarityBoost float64
	Style           float64
	UseSpeakerBoost bool
}

// TTSSettings resolves the top-level tts block, applying the package defaults.
func (c *Config) TTSSettings() TTSSettings {
	s := TTSSettings{
		Engine:  DefaultTTSEngine,
		Bitrate: DefaultTTSBitrate,
		// Loudness normalisation is on by default: raw engine output is ~9 dB
		// quieter than the rest of the corpus and varies by up to 3 LU between
		// sentences, which is audible over the mower.
		LoudNorm: true,
	}
	if c.TTS.Engine != "" {
		s.Engine = c.TTS.Engine
	}
	if c.TTS.Bitrate != "" {
		s.Bitrate = c.TTS.Bitrate
	}
	if c.TTS.LoudNorm != nil {
		s.LoudNorm = *c.TTS.LoudNorm
	}

	if s.Engine == TTSEngineCartesia {
		// Cartesia has no counterpart of the ElevenLabs voice knobs, so they stay
		// zero - which also keeps them out of the render identity.
		s.Model = DefaultCartesiaModel
		if c.TTS.Cartesia.Model != "" {
			s.Model = c.TTS.Cartesia.Model
		}
		s.Voice = c.TTS.Cartesia.Voice
		return s
	}

	el := c.TTS.ElevenLabs
	s.Model = DefaultTTSModel
	if el.Model != "" {
		s.Model = el.Model
	}
	s.Voice = el.Voice
	s.Stability = DefaultTTSStability
	s.SimilarityBoost = DefaultTTSSimilarityBoost
	s.Style = DefaultTTSStyle
	s.UseSpeakerBoost = DefaultTTSUseSpeakerBoost
	if el.Stability != nil {
		s.Stability = *el.Stability
	}
	if el.SimilarityBoost != nil {
		s.SimilarityBoost = *el.SimilarityBoost
	}
	if el.Style != nil {
		s.Style = *el.Style
	}
	if el.UseSpeakerBoost != nil {
		s.UseSpeakerBoost = *el.UseSpeakerBoost
	}
	return s
}
