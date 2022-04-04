NUA:
1) 2021-01-18: stable-baselines3 and imitation pkgs have version mismatch. SB3 should be the latest version. Install imitation with PYPI release but change imitation pkg in ~/.local/lib/python3.8/site-packages/ with the latest commit (git cloned) imittation pkg.

sudo cp -r imitation/src/imitation ~/.local/lib/python3.8/site-packages/