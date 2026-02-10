# y3-robotics-cw
Robotics coursework @ Imperial College London

# Running sync file watcher

Here is how you copy things to remote via SSH while respecting `.gitignore`:

```sh
rsync -az --delete --filter=":- .gitignore" --password-file="./.rsync_passwd" ./ pi@{{IP}}::prac-files/y3-robotics-cw-{{USER}}/
```

Or periodically:

```sh
watch -n 1 'rsync -az --delete --filter=":- .gitignore" --password-file="./.rsync_passwd" ./ pi@{{IP}}::prac-files/y3-robotics-cw-{{USER}}/'
```
Or with JUST

```sh
# set USER explicitly
just watch-rsync {{IP}} {{USER}}

# set `export RSYNC_USER={{USER}}` in .envrc
just watch-rsync {{IP}}
```
