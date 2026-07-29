# Farham LRPTW instances

- Source: https://gitlab.com/pharham/test-instances
- Upstream path: `LRPTW/Set1.zip`
- Upstream commit: `4a4b5dc60091d42d0ee0bd93dc11aa6d9459c050`
- Downloaded: 2026-07-27
- SHA-256 (`Set1.zip`): `81d723952d69ac42356342307cdc50d8d2d73dfa5b7cb097c03c61d90581d221`

`Set1.zip` is retained as downloaded. The 138 contained LRPTW instance files
are also extracted into this directory for direct use.

For this project's `clrp-tw` mode, both travel cost and travel time use
`floor(Euclidean distance)`, matching the published Farham LRPTW results.
The active customer count is the prefix declared by `NUM_CUSTOMER`; several
files physically retain additional Solomon customer rows. `NUM_CDC` must be
zero because the solver supports the single-echelon LRPTW files, not the
two-echelon instances.

Model/benchmark semantics were checked against:

- Farham, Süral, and İyigün (2018),
  *A column generation approach for the location-routing problem with time
  windows*, Computers & Operations Research 90, 249–263;
- Farham (2020), [PhD thesis](https://etd.lib.metu.edu.tr/upload/12625395/index.pdf).

The upstream project does not include a clearly identified data license in
this snapshot; retain this provenance when redistributing the files.
