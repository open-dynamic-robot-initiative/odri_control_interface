{
  description = "odri-control-interface";

  inputs = {
    # TODO: drop `/module` after https://github.com/Gepetto/nix/pull/54
    gepetto.url = "github:gwennlbh/gepetto-nix/odri";
    flake-parts.follows = "gepetto/flake-parts";
    nixpkgs.follows = "gepetto/nixpkgs";
    nix-ros-overlay.follows = "gepetto/nix-ros-overlay";
    systems.follows = "gepetto/systems";
    treefmt-nix.follows = "gepetto/treefmt-nix";
    utils.url = "github:Gepetto/nix-lib";
  };

  outputs =
    inputs:
    inputs.flake-parts.lib.mkFlake { inherit inputs; } {
      systems = import inputs.systems;
      imports = [ inputs.gepetto.flakeModule ];
      perSystem =
        {
          lib,
          pkgs,
          self',
          ...
        }:
        {
          packages = {
            default = self'.packages.odri-control-interface;
            odri-control-interface = pkgs.odri-control-interface.overrideAttrs {
              version = inputs.utils.lib.rosVersion pkgs ./package.xml;
              src = lib.fileset.toSource {
                root = ./.;
                fileset = lib.fileset.unions [
                  ./demos
                  ./include 
                  ./src
                  ./srcpy
                  ./CMakeLists.txt
                  ./package.xml
                ];
              };
            };
          };

          apps = {
            testbench = {
              type = "app";
              program = "${self'.packages.odri-control-interface}/bin/odri_control_interface_demo_testbench";
            };
            solo12 = {
              type = "app";
              program = "${self'.packages.odri-control-interface}/bin/odri_control_interface_demo_solo12";
            };
          };
        };
    };
}
