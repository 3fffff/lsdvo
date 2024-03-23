module.exports = {
  entry: {
    EntryPoint: './dist/index.js'
  },
  devtool: 'source-map',
  output: {
    filename: 'bundle.js',
    library: "LSD",
  },
};