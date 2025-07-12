group "default" {
  targets = ["openmower"]
}

target "openmower" {
  context = "."
  dockerfile = "docker/Dockerfile"
  platforms = ["linux/amd64", "linux/arm64"]
  tags = [
    "ghcr.io/pepeuch/openmower:mowgli-v22.06.25",
    "ghcr.io/pepeuch/openmower:mowgli-latest"
  ]
  output = ["type=registry"]
  args = {
    OM_VERSION = "mowgli-v22.06.25"
  }
  labels = {
    "org.opencontainers.image.title"       = "OpenMower Mowgli"
    "org.opencontainers.image.version"     = "v22.06.25"
    "org.opencontainers.image.description" = "Build for OpenMower Mowgli platform"
  }
}