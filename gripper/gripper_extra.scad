diameter = 9.5;
thickness = 2;

clip(max_x=3)
  rotate_extrude(360)
  polygon(concat(
    [for (i = [0:15]) [diameter/2 + (i % 2)/5, i / 3]],
    [[diameter/2 + thickness, 15 / 3], [diameter/2 + thickness, 0]],
  ));

translate([-8.5, -2, 0])
  cube([3, 4, 15 / 3]);
