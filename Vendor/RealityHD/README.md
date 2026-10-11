# RealityHD (vendored)

A subset of [RealityHD](https://github.com/hunterh37/RealityHD) at commit `c01e31b63b5cbd0f67daf5401247d81149f24303`, MIT-licensed (see `LICENSE`). It contains only the library targets `RealCore`, `RealMaterials`, `RealKit` and `RealLibrary`.

It is vendored, rather than fetched as a package, because Xcode 26.0 and 26.1 fail to type-check 20 long expressions in 18 files. Each of those expressions is split into typed sub-expressions with identical math and marked `// FlyFree patch:`. Find them with:

```sh
grep -rn "FlyFree patch" Sources
```

See `docs/ASSETS.md` in FlyFree for how to update the copy or switch back to the upstream package.
