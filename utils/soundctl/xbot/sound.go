package xbot

import (
	"context"
	"time"
)

// SoundService register/input IDs (see services/sound_service.json).
const (
	soundRegisterDefinitions = 0
	soundInputVolume         = 0
)

// SoundService is a thin client for the SoundService (service id 13).
type SoundService struct {
	c *Conn
}

// NewSoundService discovers and claims the SoundService.
func NewSoundService(ctx context.Context, bindIP string, heartbeat time.Duration) (*SoundService, error) {
	c, err := Dial(ctx, ServiceSound, bindIP, heartbeat)
	if err != nil {
		return nil, err
	}
	return &SoundService{c: c}, nil
}

// Close closes the underlying connection.
func (s *SoundService) Close() error { return s.c.Close() }

// SetDefinitions sends the sound-definitions blob (already heatshrink-encoded)
// as the "Sound Definitions" register. The firmware treats a register update as
// a reconfiguration (it restarts the service) and persists it to flash.
func (s *SoundService) SetDefinitions(blob []byte) error {
	body := append(PackDescriptor(soundRegisterDefinitions, uint32(len(blob))), blob...)
	return s.c.send(MsgTransaction, 1 /* configuration transaction */, 0, body)
}

// SetVolume sends the master volume (0..100) as the "Volume" input. The service
// must be running (i.e. configured) for the input to be accepted.
func (s *SoundService) SetVolume(volume uint8) error {
	return s.c.send(MsgData, 0, soundInputVolume, []byte{volume})
}
