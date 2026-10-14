package main

import (
	"fmt"
	"os"

	"github.com/ClemensElflein/open_mower_ros/utils/soundctl/internal/cli"
)

func main() {
	if err := cli.Execute(); err != nil {
		fmt.Fprintln(os.Stderr, "Error:", err)
		os.Exit(1)
	}
}
