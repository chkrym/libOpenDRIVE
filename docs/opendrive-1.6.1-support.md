# ASAM OpenDRIVE 1.6.1 support

This table maps the sections of the ASAM OpenDRIVE 1.6.1 specification to what libOpenDRIVE does with them.

Status values:

- **Implemented**: parsed and used for geometry, meshes or routing.
- **Parse-only**: parsed into the data model and available through the API, but not evaluated further. Mostly for elements the specification gives no geometry to, or that reference external data.
- **Not supported**: ignored when reading a file.

Informative sections (introduction, conventions, coordinate system overview, use cases) are left out.

## 4 General architecture

| Section | Elements | Status | Notes | PR |
|---|---|---|---|---|
| 4.1 File structure | `.xodr`, `.xodrz` | Implemented | Gzip-compressed files are detected by their magic bytes and need zlib (`OPENDRIVE_WITH_ZLIB`, default ON). | #23 |
| 4.2 Combining files | `<include>` | Implemented | Paths are relative to the including file. Content is merged where the element appears, nested includes are resolved and recursive ones skipped. | #20 |
| 4.3.2 Header | `<header>` | Parse-only | `OpenDriveMap::header`: revision, name, version, date, vendor, bounds. | #5 |
| 4.4.1 Traffic direction | `<road @rule>` | Implemented | `LHT` reverses the driving direction of the lanes in the routing graph. | #24 |

## 5 Additional data

| Section | Elements | Status | Notes | PR |
|---|---|---|---|---|
| 5.1 User data | `<userData>` | Parse-only | Kept on every element as `AdditionalData` (code, value, serialized content). | #12, #27 |
| 5.2 Including data | `<include>` | Implemented | See 4.2. The `<include>` elements stay listed in `AdditionalData::includes`. | #12, #20 |
| 5.3 Layout types | | Not supported | | |
| 5.4 Data quality | `<dataQuality>` | Parse-only | `error` and `rawData`. | #12, #27 |

## 6 Coordinate systems

| Section | Elements | Status | Notes | PR |
|---|---|---|---|---|
| 6.6 Georeferencing | `<geoReference>`, `<offset>` | Parse-only | `proj4` string and `Header::offset`. No projection is performed. | #5 |

## 7 Geometry

| Section | Elements | Status | Notes | PR |
|---|---|---|---|---|
| 7.2 Straight line | `<line>` | Implemented | | upstream |
| 7.3 Spiral | `<spiral>` | Implemented | | upstream |
| 7.4 Arc | `<arc>` | Implemented | | upstream |
| 7.6 Cubic polynom (deprecated) | `<poly3>` | Implemented | Converted to a normalized parametric cubic curve. | #3 |
| 7.7 Parametric cubic curve | `<paramPoly3>` | Implemented | | upstream |

## 8 Roads

| Section | Elements | Status | Notes | PR |
|---|---|---|---|---|
| 8.2 Road linkage | `<link>`, `<predecessor>`, `<successor>`, `<neighbor>` | Implemented | `@elementS` / `@elementDir` links are followed into the lane section at that position. | upstream, #26 |
| 8.3 Road type | `<type>`, `<speed>` | Parse-only | Includes `@country`. Speed defaults to m/s when no unit is given. | upstream, #8, #14 |
| 8.4.1 Road elevation | `<elevationProfile>` | Implemented | | upstream |
| 8.4.2 Superelevation | `<superelevation>` | Implemented | `<crossfall>` (1.4) is still read. | upstream |
| 8.4.3 Shape | `<shape>` | Implemented | Added to the surface height and normal. Lane meshes are subdivided along t to follow it. | #4, #6, #21 |
| 8.5 Road surface | `<surface><CRG>` | Parse-only | `Road::surface_crgs`. The CRG data itself is not read. | #12 |

## 9 Lanes

| Section | Elements | Status | Notes | PR |
|---|---|---|---|---|
| 9.2 Lane sections | `<laneSection @singleSide>` | Implemented | Lanes of the other side carry over from the previous section. | upstream, #2 |
| 9.3 Lane offset | `<laneOffset>` | Implemented | | upstream |
| 9.4 Lane linkage | lane `<predecessor>` / `<successor>` | Implemented | Several links per lane (splits and merges) are kept in `Lane::predecessors` / `successors` and used for routing. | upstream, #15 |
| 9.5.1 Lane width | `<width>` | Implemented | | upstream |
| 9.5.2 Lane borders | `<border>` | Implemented | Used when a lane group has no `<width>`. | #1 |
| 9.5.3 Lane type | `@type` | Parse-only | | upstream |
| 9.5.4 Lane material | `<material>` | Parse-only | | #7 |
| 9.5.5 Lane speed limit | `<speed>` | Parse-only | | #7 |
| 9.5.6 Lane access | `<access>` | Parse-only | | #7 |
| 9.5.7 Lane height | `<height>` | Implemented | Applied when `with_lane_height` is set. | upstream |
| 9.5.8 Excluding lanes from superelevation | `@level` | Implemented | Level lanes skip superelevation and shape. | #18 |
| 9.6 Road markings | `<roadMark>` | Implemented | Simple `@type` values get default line patterns (broken: 3 m line, 6 m gap; double lines). | upstream, #22 |
| 9.6.1 Road marking types and lines | `<type><line>` | Implemented | Per-line `@color` is exposed on the road mark mesh. | upstream, #17 |
| 9.6.2 Explicit road marking | `<explicit><line>` | Implemented | | #11 |
| 9.6.3 Offset in road markings | `<sway>` | Implemented | | #11 |
| 9.7 Specific lane rules | `<rule>` | Parse-only | Free-text values. | #7 |

## 10 Junctions

| Section | Elements | Status | Notes | PR |
|---|---|---|---|---|
| 10.1, 10.2 Incoming and connecting roads | `<connection>`, `<laneLink>` | Implemented | Used for routing. | upstream |
| 10.2.1 Priorities | `<priority>` | Parse-only | Not used for routing. | upstream |
| 10.3 Road surface within junctions | `<surface><CRG>` | Parse-only | `Junction::surface_crgs`. | #12 |
| 10.4 Virtual junctions | `<junction @type="virtual">` | Implemented | Regular connections branch off the main road at `@elementS` of the link, or else at the closest point of its reference line. | #8, #26 |
| 10.4.1 Virtual connections | `<connection @type="virtual">` | Implemented | Add routing edges from the predecessor road to the successor road. | #8, #26 |
| 10.5 Junction groups | `<junctionGroup>` | Parse-only | | #8 |
| 10.5.1 Controllers for junctions | `<controller>` in `<junction>` | Parse-only | | upstream |

## 11 Objects

Objects are read only when `with_road_objects` is set.

| Section | Elements | Status | Notes | PR |
|---|---|---|---|---|
| 11 Objects | `<object>` | Implemented | Box or cylinder mesh from the bounding attributes. | upstream |
| 11.1 Repeating objects | `<repeat>` | Implemented | Length and radius are interpolated along the repeat. | upstream, #16 |
| 11.2 Object outline | `<outline>`, `<cornerRoad>`, `<cornerLocal>` | Implemented | An outline replaces the bounding box and follows repeats. Open outlines are drawn as walls. A closed outer outline with inner outlines is treated as an envelope and only the inner outlines are drawn. | upstream, #28 |
| 11.3 Object material | `<material>` | Parse-only | | #9 |
| 11.4 Lane validity | `<validity>` | Parse-only | | upstream, #9 |
| 11.5 Parking spaces | `<parkingSpace>` | Parse-only | | #9 |
| 11.6 Object marking | `<markings>` | Implemented | Flat strips along a bounding box side or between outline corners, dashed by `@lineLength` / `@spaceLength`. | #9, #29 |
| 11.7 Object borders | `<borders>` | Implemented | Flat strips on top of the outline. | #9, #29 |
| 11.8 Object reference | `<objectReference>` | Parse-only | No geometry in the specification. | #9 |
| 11.9 Tunnels | `<tunnel>` | Parse-only | No geometry in the specification. | #9 |
| 11.10 Bridges | `<bridge>` | Parse-only | No geometry in the specification. | #9 |

## 12 Signals

| Section | Elements | Status | Notes | PR |
|---|---|---|---|---|
| 12 Signals | `<signal>` | Implemented | Box mesh. With `@orientation="-"` it faces negative s. | upstream, #25 |
| 12.1 Lane validity | `<validity>` | Parse-only | | upstream |
| 12.2 Signal dependency | `<dependency>` | Parse-only | | #10 |
| 12.3 Links between signals and objects | `<reference>` | Parse-only | | #10 |
| 12.4 Signal positioning | `<positionInertial>`, `<positionRoad>` | Implemented | The mesh is placed at the physical position. s/t stay at the logical position. | #10, #19 |
| 12.5 Reuse of signal information | `<signalReference>` | Parse-only | | #10 |
| 12.6 Controllers | `<controller>`, `<control>` | Parse-only | | #10 |

## 13 Railroads

| Section | Elements | Status | Notes | PR |
|---|---|---|---|---|
| 13.1 Railroad tracks | | Parse-only | Tracks are read as roads. | upstream |
| 13.2 Switches | `<switch>`, `<mainTrack>`, `<sideTrack>`, `<partner>` | Parse-only | `Road::id_to_switch`. | #13 |
| 13.3 Stations | `<station>`, `<platform>`, `<segment>` | Parse-only | `OpenDriveMap::id_to_station`. | #13 |
