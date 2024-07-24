# For the CUSTOM hardware, source the environment from the home dir (if exists)
if [[ -f ~/mower_params/default_environment.sh ]]; then
  echo "sourcing custom default environment in ~/mower_params/default_environment.sh"
  source ~/mower_params/default_environment.sh
else
  echo "No custom default environment file in: ~/mower_params/default_environment.sh"
fi
