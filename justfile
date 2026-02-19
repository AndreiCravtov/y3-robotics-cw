# Default command when 'just' is run without arguments
default:
  @just --list

# Lint files
[group('Main')]
lint:
  nix fmt && uv run ruff check --fix src

# Format files
[group('Main')]
fmt:
  nix fmt && uv run ruff format src

# Run a single rsync over SSH
[group('Pi Devops')]
rsync IP=env('PI_IP') USER=env('RSYNC_USER'):
  #!/usr/bin/env bash
  set -euo pipefail

  mkdir -p pi_ssh
  KEY="pi_ssh/id_ed25519"
  chmod 600 "$KEY"

  SSH_OPTS=(
    -i "$KEY"
    -o IdentitiesOnly=yes
    -o StrictHostKeyChecking=no
    -o UserKnownHostsFile=/dev/null
    -o ControlMaster=auto
    -o ControlPersist=5m
    -o ControlPath=pi_ssh/cm-%C
  )

  rsync -a --delete --filter=":- .gitignore" \
    -e "ssh ${SSH_OPTS[*]}" \
    ./ "pi@{{IP}}:/home/pi/prac-files/y3-robotics-cw-{{USER}}/"

# Watch the sync by repeatedly invoking the recipe above
[group('Pi Devops')]
watch-rsync IP=env('PI_IP') USER=env('RSYNC_USER') INTERVAL="1":
  watch -n {{INTERVAL}} -- just rsync {{IP}} {{USER}}

# Set the current PI address for this session
[group('Pi Devops')]
set-pi-ip IP:
  echo "export PI_IP={{IP}}" > .envrc.session && direnv reload

# Update nix flake
[group('Nix')]
update-nix:
  nix flake update

# Show flake outputs
[group('Nix')]
show-nix:
  nix flake show --all-systems --legacy

# Enter Nix REPL
[group('Nix')]
repl-nix:
  nix repl .

# Run typechecking
check-py:
  uv run basedpyright --project pyproject.toml

# Sync packages
[group('Python')]
sync-py:
  uv sync --all-packages

# Forcefully sync packages
[group('Python')]
sync-clean-py:
  uv sync --all-packages --force-reinstall --no-cache