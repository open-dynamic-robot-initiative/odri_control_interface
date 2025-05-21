{
  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    utils.url = "github:Gepetto/nix-lib";

    odri-masterboard-sdk = {
      # FIXME update after https://github.com/open-dynamic-robot-initiative/master-board/pull/173
      url = "git+https://github.com/gwennlbh/master-board?ref=nix&rev=d5d6105f8db1770cd07fdde300c4c008e36de818";
      inputs.nixpkgs.follows = "nixpkgs";
    };
  };

  outputs =
    {
      nixpkgs,
      utils,
      odri-masterboard-sdk,
      ...
    }:
    let
      pkgs = nixpkgs.legacyPackages.x86_64-linux;
      sdk = odri-masterboard-sdk.packages.x86_64-linux.default;
      rosVersion = utils.lib.rosVersion pkgs;
    in
    {
      packages.x86_64-linux.default = pkgs.stdenv.mkDerivation rec {
        pname = "odri-control";
        version = rosVersion ./package.xml;

        src = builtins.path {
          name = pname;
          path = ./.;
        };

        nativeBuildInputs =
          [ sdk ]
          ++ (with pkgs; [
            cmake
            yaml-cpp
            eigen
            python312Packages.eigenpy
            python312Packages.boost
            python312
          ]);
      };
    };
}
