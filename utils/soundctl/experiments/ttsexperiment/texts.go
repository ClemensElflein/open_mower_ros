package main

// phrase is one line of the sound set that shall be spoken.
type phrase struct {
	ID   string
	Text string
}

// englishPhrases is taken from Firmware/LowLevel/soundfiles/01/GoogleTTS_Strings.txt.
// Only the obvious typos were fixed so that no engine stumbles over them
// ("autononous" -> "autonomous", "save" -> "safe", "ups" -> "Oops",
// "rtk" -> "RTK"), sentences are capitalised and end with a full stop /
// exclamation mark because prosody depends on punctuation.
var englishPhrases = []phrase{
	{"001", "Hi, I am Steve. Your Open Mower green care is my profession."},
	{"002", "Open Mower start up successful."},
	{"003", "Initializing robot operating system."},
	{"004", "Starting map area recording."},
	{"005", "Waiting for RTK GPS signal."},
	{"006", "Calculating map dimensions."},
	{"007", "Initiate mowing sequence."},
	{"008", "Stop button triggered."},
	{"009", "Emergency wheel lift sensor triggered."},
	{"010", "Rain detected, heading back to base."},
	{"011", "Open Mower has completed mowing the lawn, heading back to charging station."},
	{"012", "Stay back, autonomous robot mower in use."},
	{"013", "Oops, I did it again."},
	{"016", "Robot operating system start up successful."},
	{"017", "Robot operating system stopped."},
	{"018", "Raspberry Pi shut down and powered off. It's safe to power off your Open Mower now."},
	{"019", "IMU initialization failed."},
	{"020", "Down."},
	{"021", "Up."},
	{"022", "Switched to English language."},
	{"023", "Emergency cleared."},
	{"024", "Emergency triggered by robot operating system."},
	{"025", "Emergency triggered by collision sensor."},
}

// germanPhrases is taken from Firmware/LowLevel/soundfiles/49/GoogleTTS_Strings.txt.
// Spelling that would trip a TTS was normalised (Rasemäher -> Rasenmäher,
// Pai -> Pi, ei-em-ju -> IMU).
var germanPhrases = []phrase{
	{"001", "Hallo, ich bin Steve. Dein intelligenter Rasenmäher, Rasenpflege ist mein Beruf."},
	{"002", "Open Mower erfolgreich gestartet."},
	{"003", "Initialisiere Roboter-Betriebssystem."},
	{"004", "Starte Aufzeichnung des Lageplans."},
	{"005", "Warte auf RTK-GPS-Signal."},
	{"006", "Berechne Abmessungen des Lageplans."},
	{"007", "Starte den Mähvorgang."},
	{"008", "Stopp-Taste gedrückt."},
	{"009", "Not-Aus durch Rad-Hebesensor."},
	{"010", "Regen festgestellt, fahre zurück zur Basis."},
	{"011", "Open Mower hat das Rasenmähen beendet und fährt zurück zur Ladestation."},
	{"012", "Vorsicht! Autonomer Mähroboter im Einsatz."},
	{"013", "Au weia, schon wieder."},
	{"016", "Start des Roboter-Betriebssystems erfolgreich."},
	{"017", "Das Roboter-Betriebssystem wurde gestoppt."},
	{"018", "Der Raspberry Pi wurde heruntergefahren. Du kannst deinen Open Mower nun ausschalten."},
	{"019", "Die IMU-Initialisierung ist fehlgeschlagen."},
	{"020", "Leiser."},
	{"021", "Lauter."},
	{"022", "Auf deutsche Sprache umgestellt."},
	{"023", "Notaus aufgehoben."},
	{"024", "Not-Aus durch Roboter-Betriebssystem."},
	{"025", "Not-Aus durch Kollisionssensor."},
}

// phrasesFor returns the phrase set for a language code ("en" or "de").
func phrasesFor(lang string) []phrase {
	if lang == "de" {
		return germanPhrases
	}
	return englishPhrases
}
