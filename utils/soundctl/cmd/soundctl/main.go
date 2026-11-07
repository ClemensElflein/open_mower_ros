package main

import (
	"os"

	"github.com/ClemensElflein/open_mower_ros/utils/soundctl/internal/cli"
)

func main() {
	if err := cli.Execute(); err != nil {
		os.Exit(1)
	}
}
