# Joint angle meter

Reads joint angles off the leg through a webcam, for the calibration sweep in
`docs/kinematics.md` section 8.

    cd tools
    python3 -m http.server 8000

then open <http://localhost:8000/angle-meter/>.

It has to be served, not opened as a file. Camera access needs a secure
context, and `file://` is not one; `localhost` is.

## Markers

| Colour | Goes on |
|--------|---------|
| green  | hip pitch axis |
| yellow | knee axis |
| blue   | foot |

So green to yellow is the thigh, yellow to blue is the shank.

## Use

1. Start the camera and point it square at the leg plane.
2. Open "marker tuning" and click "pick" next to each colour, then click that
   dot in the video. Widen the tolerance until it tracks without grabbing
   anything else. The pixel counts at the bottom say how well each is seen.
3. Set the vertical reference: "Set by drawing a line", then drag along
   something you know is upright. Only the absolute angles need this.
4. Wait for "steady", then copy the `pt <angle>` line straight into the
   calibration sketch's serial monitor.

## What each number is

- **thigh** and **shank**, measured from straight down. Absolute, so they
  depend on the vertical reference being right.
- **t3**, the knee, as shank minus thigh. A difference of two angles, so
  camera roll cancels out of it completely and the vertical reference does not
  matter. This is the one to use for the knee.
- **hip to foot**, green to blue. Useful for the hip roll from a front view,
  where the thigh and shank are foreshortened but the whole leg is not.

Tick "forward is left" if the robot faces left in frame, so that positive
still means forward.

## Accuracy

Centroid error against a synthetic frame is under a pixel, and the angles come
out within 0.03 degrees of truth. The real limits are elsewhere: the camera
being square to the leg plane, and how precisely the stickers sit on the joint
centres. Fifteen degrees of camera misalignment costs about one degree; thirty
costs four. A sticker offset from the joint centre is a constant error in that
link's angle and does not average out.
