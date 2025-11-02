# FTC DECODE Field Positions Diagram

## Field Layout (144" × 144" / 12' × 12')

```
  0    24   48   72   96   120  144 (inches)
  ┌────┬────┬────┬────┬────┬────┐
0 │ RL │    │    │    │    │ RR │  Red Alliance Zone
  │(12,│    │    │    │    │(132│  (Y: 0-24")
  │ 12)│    │    │    │    │,12)│
  ├────┼────┼────┼────┼────┼────┤
24│    │    │    │    │    │    │
  │    │    │    │    │    │    │
  │    │    │    │    │    │    │
  ├────┼────┼────┼────┼────┼────┤
48│    │    │    │    │    │    │
  │    │    │    │    │    │    │
  │    │    │    │    │    │    │
  ├────┼────┼────┼────┼────┼────┤
72│    │    │    │ CC │    │    │  Center Field
  │    │    │    │(72,│    │    │  (72, 72)
  │    │    │    │ 72)│    │    │
  ├────┼────┼────┼────┼────┼────┤
96│    │    │    │    │    │    │
  │    │    │    │    │    │    │
  │    │    │    │    │    │    │
  ├────┼────┼────┼────┼────┼────┤
120│   │    │    │    │    │    │
  │    │    │    │    │    │    │
  │    │    │    │    │    │    │
  ├────┼────┼────┼────┼────┼────┤
144│ BL │    │    │    │    │ BR │  Blue Alliance Zone
  │(12,│    │    │    │    │(132│  (Y: 120-144")
  │132)│    │    │    │    │,132│
  └────┴────┴────┴────┴────┴────┘
```

## Starting Positions Legend

### Red Alliance Positions
- **RL (Red Left)**: (12", 12") - Bottom left corner
- **RR (Red Right)**: (132", 12") - Bottom right corner

### Blue Alliance Positions  
- **BL (Blue Left)**: (12", 132") - Top left corner
- **BR (Blue Right)**: (132", 132") - Top right corner

### Key Field Locations
- **CC (Center)**: (72", 72") - Center field target
- **Scoring Zones**:
  - Red Scoring: Y = 24" (one tile from red edge)
  - Blue Scoring: Y = 120" (one tile from blue edge)

## Field Zones and Navigation

```
┌─────────────────────────────────────────────┐
│                BLUE ZONE                    │ ← Blue Alliance (Y: 120-144")
│  BL(12,132)              BR(132,132)       │
├─────────────────────────────────────────────┤
│                                             │
│                MID FIELD                    │
│                                             │
│            CENTER(72,72)                    │ ← Target Return Position
│                                             │
│                                             │
├─────────────────────────────────────────────┤
│                RED ZONE                     │ ← Red Alliance (Y: 0-24")
│  RL(12,12)               RR(132,12)        │
└─────────────────────────────────────────────┘
```

## Tile Grid System (24" × 24" tiles)

```
Tile Coordinates (6×6 grid):
┌───┬───┬───┬───┬───┬───┐
│0,5│1,5│2,5│3,5│4,5│5,5│  ← Blue Zone
├───┼───┼───┼───┼───┼───┤
│0,4│1,4│2,4│3,4│4,4│5,4│
├───┼───┼───┼───┼───┼───┤
│0,3│1,3│2,3│3,3│4,3│5,3│
├───┼───┼───┼───┼───┼───┤
│0,2│1,2│2,2│3,2│4,2│5,2│
├───┼───┼───┼───┼───┼───┤
│0,1│1,1│2,1│3,1│4,1│5,1│
├───┼───┼───┼───┼───┼───┤
│0,0│1,0│2,0│3,0│4,0│5,0│  ← Red Zone
└───┴───┴───┴───┴───┴───┘
```

## Robot Mission Path (Example from Blue Left)

```
START: BL(12,132) → Search Pattern → Ball Locations → AprilTag → CENTER(72,72)

1. Ball Search: Rotating scan from starting position
2. Ball Collection: Drive to detected balls (location varies)
3. AprilTag Search: Scan for AprilTag ID 20
4. Shooting Position: 24" from AprilTag
5. Return Path: Navigate to center field (72,72)
```

## Distance and Movement Specifications

### Detection Ranges
- **Ball Detection**: 12" maximum range
- **Ball Pickup**: 4" proximity required
- **AprilTag Approach**: 24" shooting distance
- **Field Center Tolerance**: 24" (one tile)

### Movement Speeds
- **Normal Drive**: 0.4 power (40%)
- **Turn/Scan**: 0.3 power (30%) 
- **Ball Search**: 0.2 power (20%)
- **Precision Approach**: 0.25 power (25%)

### Robot Dimensions
- **Width**: 18" (1.5 tiles)
- **Length**: 18" (1.5 tiles)
- **Turning Radius**: Mecanum drive (zero-point turn capable)

## Coordinate System Notes

- **Origin (0,0)**: Bottom-left corner (Red Left area)
- **X-Axis**: Left to Right (0" to 144")
- **Y-Axis**: Bottom to Top (0" to 144") 
- **Heading**: 0° = facing toward opposite alliance
- **Field Bounds**: Robot position kept within 0-144" range

## Current Configuration
- **Default Start**: Blue Left (12", 132")
- **Target**: AprilTag ID 20 (Blue alliance)
- **Mission**: Collect 3 balls → Fire 3 shots → Return to center