# RE-RASSOR using Pixi + RoboStack Approach
# This avoids nix-ros-overlay entirely!
{
  inputs = {
    nixpkgs.url = "github:nixos/nixpkgs/nixos-unstable";
    flake-parts.url = "github:hercules-ci/flake-parts";
    systems.url = "github:nix-systems/default";
    devshell.url = "github:numtide/devshell";
  };

  outputs = inputs@{ flake-parts, systems, devshell, ... }:
    flake-parts.lib.mkFlake { inherit inputs; } {
      systems = import systems;
      imports = [ devshell.flakeModule ];

      perSystem = { pkgs, ... }:
        let
          inherit (pkgs.lib) optionalString;
          isDarwin = pkgs.stdenv.isDarwin;

          # Colcon build configuration
          colconDefaults = pkgs.writeText "defaults.yaml" ''
            build:
              cmake-args:
                - -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
                - -DPython_FIND_VIRTUALENV=ONLY
                - -DPython3_FIND_VIRTUALENV=ONLY
                - -Wno-dev
                ${optionalString isDarwin "- -DCMAKE_BUILD_WITH_INSTALL_RPATH=ON"}
          '';
        in
        {
          devshells.default = {
            env = [
              {
                name = "COLCON_DEFAULTS_FILE";
                value = toString colconDefaults;
              }
            ];

            devshell = {
              packages = with pkgs; [
                pixi        # Pixi package manager
                git
                cmake
                gnumake
              ];

              startup.activate.text = ''
                echo "🤖 RE-RASSOR Development Environment"
                echo "════════════════════════════════════════════════════"
                echo ""
                echo "This environment uses:"
                echo "  • Pixi for package management"
                echo "  • RoboStack for ROS2 Humble packages"
                echo "  • Pre-built binaries (no compilation needed!)"
                echo ""

                if [ -f pixi.toml ]; then
                  echo "Activating Pixi environment..."
                  ${optionalString isDarwin ''
                    export DYLD_FALLBACK_LIBRARY_PATH="$PWD/.pixi/envs/default/lib:$DYLD_FALLBACK_LIBRARY_PATH"
                  ''}
                  eval "$(pixi shell-hook)";
                  echo "✓ ROS2 Humble environment ready!"
                  echo ""
                  echo "Next steps:"
                  echo "  1. git clone https://github.com/evahocking/RERASSOR-Simulation.git ros2_ws"
                  echo "  2. cd ros2_ws && colcon build"
                  echo "  3. source install/setup.bash"
                else
                  echo "⚠ No pixi.toml found. Creating initial configuration..."
                  echo ""
                  echo "Run: pixi init && pixi add ros-humble-desktop"
                fi

                echo "════════════════════════════════════════════════════"
              '';

              motd = "";
            };
          };
        };
    };
}
