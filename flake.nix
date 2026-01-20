{
  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-parts = {
      url = "github:hercules-ci/flake-parts";
      inputs.nixpkgs-lib.follows = "nixpkgs";
    };
    make-shell.url = "github:nicknovitski/make-shell";
  };
  outputs = inputs @ {flake-parts, ...}:
    flake-parts.lib.mkFlake {inherit inputs;} (
      {
        flake-parts-lib,
        self,
        ...
      }: {
        imports = [inputs.make-shell.flakeModules.default];
        systems = [
          "x86_64-linux"
          "aarch64-darwin"
        ];

        debug = true; # Enable options autocompletion

        perSystem = {
          config,
          self',
          inputs',
          pkgs,
          lib,
          system,
          ...
        }: {
          # "Flake parts does not yet come with an endorsed module that initializes the pkgs argument."
          # So we must do this manually; https://flake.parts/overlays#consuming-an-overlay
          _module.args.pkgs = import inputs.nixpkgs {
            inherit system;
            config = {
              allowBroken = true;
              allowUnsupportedSystem = true;
              allowUnfree = true;
            };
            #overlays = ADD OVERLAYS HERE;
          };

          formatter = pkgs.alejandra; # For 'nix fmt'

          make-shells.default = {
            name = "jupyter-notebook-shell";
            packages = with pkgs; [
              git
              just
              nixd
              python313
              uv
              ruff
              basedpyright
              nodejs
            ];

            # Arguments which are intended to be environment variables in the shell environment
            # should be changed to attributes of the `env` option
            env = {
              LD_LIBRARY_PATH = lib.makeLibraryPath (with pkgs; [
                stdenv.cc.cc
                python313
              ]);
            };
          };
        };
      }
    );
}
