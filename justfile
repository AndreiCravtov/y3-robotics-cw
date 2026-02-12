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

# Sync files with remote on PI
[group('Pi Devops')]
watch-rsync IP=env('PI_IP') USER=env('RSYNC_USER'):
  chmod 600 ./.rsync_passwd && watch -n 1 'rsync -az --delete --filter=":- .gitignore" --password-file="./.rsync_passwd" ./ pi@{{IP}}::prac-files/y3-robotics-cw-{{USER}}/'

# Sync files with remote on PI
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