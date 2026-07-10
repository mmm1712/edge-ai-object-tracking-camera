$fn = 64;
eps = 0.01;

show_part = "assembly";
/*
  Options:
  "assembly"
  "base"
  "base_cover"
  "yoke"
  "camera_head"
  "faceplate_insert"
*/

sv_body_w = 12.2;
sv_body_d = 28.5;
sv_tab_l = 32.5;
sv_screw_d = 2.4;
sv_screw_sp = 27.0;
sv_shaft_off = 5.5;
sv_boss_d = 14.0;
sv_boss_nose = 5.5;

cam_screw_sp_x = 21.0;
cam_screw_sp_z = 12.5;
cam_hole_z_off = 5.0;
cam_screw_d = 2.3;
lens_d = 15.0;
fpc_w = 21.0;

base_w = 56;
base_d = 42;
base_h = 33.8;
deck_t = 2.4;
yoke_axis_z = 33;
arm_in_x = 19;
head_w = 34;
head_d = 16.5;
head_h = 32;

side_pod_h = 40.0;
side_pod_y = 17.8;
side_pod_r = 6.4;
side_pod_zc = 30.5;

base_cover_tab_clear = 0.65;
base_cover_tab_h = 15.0;
base_cover_tab_lead_h = 2.2;
base_cover_tab_x = 13.0 - base_cover_tab_clear;
base_cover_tab_y = 7.0 - base_cover_tab_clear;
base_cover_tab_lead_x = base_cover_tab_x - 0.8;
base_cover_tab_lead_y = base_cover_tab_y - 0.8;

wall_t = 13;
module soft_box(size=[10,10,10], r=3) {
    x=size[0]; y=size[1]; z=size[2];
    hull()
    for (xx=[-x/2+r, x/2-r])
    for (yy=[-y/2+r, y/2-r])
    for (zz=[-z/2+r, z/2-r])
        translate([xx,yy,zz]) sphere(r=r);
}

module rounded_box(size=[10,10,10], r=2) {
    x=size[0]; y=size[1]; z=size[2];
    hull()
    for (xx=[-x/2+r, x/2-r])
    for (yy=[-y/2+r, y/2-r])
        translate([xx,yy,0]) cylinder(h=z, r=r, center=true);
}


module rounded_plate_xz(w=20, z=20, t=1.0, r=4) {
    hull()
    for (xx=[-w/2+r, w/2-r])
    for (zz=[-z/2+r, z/2-r])
        translate([xx, 0, zz])
            rotate([90,0,0]) cylinder(r=r, h=t, center=true);
}

module hole_x(d=3, h=20) { rotate([0,90,0]) cylinder(d=d, h=h, center=true); }

module hole_y(d=3, h=20) { rotate([90,0,0]) cylinder(d=d, h=h, center=true); }

module hole_z(d=3, h=20) { cylinder(d=d, h=h, center=true); }

module boss_clear_z(d=14, nose_d=5.5, nose_off=-5.5, h=12) {
    hull() {
        hole_z(d=d, h=h);
        translate([nose_off,0,0]) hole_z(d=nose_d, h=h);
    }
}

module base() {
    difference() {
        translate([0,0,base_h/2])
            soft_box([base_w, base_d, base_h], r=10);

        translate([-sv_shaft_off, 0, (base_h - deck_t)/2 - 0.1])
            rounded_box([sv_tab_l + 1.0, sv_body_w + 1.0, base_h - deck_t + 0.2], r=1.0);

        translate([0,0,base_h - deck_t/2])
            boss_clear_z(d=sv_boss_d, nose_d=sv_boss_nose, nose_off=-6.0, h=deck_t + 8);

        for (xx = [-sv_screw_sp/2, sv_screw_sp/2]) {
            translate([-sv_shaft_off + xx, 0, base_h/2])
                hole_z(d=sv_screw_d, h=base_h + 4);
            translate([-sv_shaft_off + xx, 0, base_h - 0.35])
                hole_z(d=5.2, h=0.8);
        }

        translate([0,0,0])
            rounded_box([46, 32, 4.4], r=9);

        for (yy = [-12, 12])
            translate([0, yy, 5])
                hole_z(d=1.7, h=10);

        for (xx = [-18, 18]) for (yy = [-12, 12])
            translate([xx, yy, 15.1])
                rounded_box([13, 7, 26], r=2);

        translate([base_w/4 + 3.0, 0, 8.5])
            rounded_box([base_w/2 + 12, 13.0, 9.0], r=2.7);

        translate([base_w/2 - 1.2, 0, 8.5])
            rounded_box([8.0, 14.0, 10.2], r=3.2);
    }
}

module base_cover_locator_tab() {

    hull() {
        translate([0,0,(base_cover_tab_h - base_cover_tab_lead_h)/2])
            rounded_box([base_cover_tab_x,
                         base_cover_tab_y,
                         base_cover_tab_h - base_cover_tab_lead_h], r=1.45);
        translate([0,0,base_cover_tab_h - base_cover_tab_lead_h/2])
            rounded_box([base_cover_tab_lead_x,
                         base_cover_tab_lead_y,
                         base_cover_tab_lead_h], r=1.2);
    }
}

module base_cover() {
    difference() {
        union() {

            translate([0,0,1.0])
                rounded_box([45.4, 31.4, 2.0], r=8.7);

            for (xx = [-18, 18]) for (yy = [-12, 12])
                translate([xx, yy, 2.0])
                    base_cover_locator_tab();
        }

        for (yy = [-12, 12]) {
            translate([0, yy, 1.0]) hole_z(d=2.3, h=6 + base_cover_tab_h);
            translate([0, yy, 0.3]) hole_z(d=4.2, h=0.7);
        }

        for (xx = [-17.5, 17.5]) for (yy = [-11, 11])
            translate([xx, yy, 0])
                hole_z(d=8, h=1.6);
    }
}

module left_minimal_arm(t=wall_t) {
    difference() {
        union() {
            hull() {

                translate([-arm_in_x - t/2, 0, 8.0])
                    rounded_box([t, 22.0, 3.6], r=1.8);

                translate([-arm_in_x - t/2, 0, yoke_axis_z])
                    soft_box([t, 17.5, 15], r=4.5);
            }

            translate([-arm_in_x + 0.6, 0, yoke_axis_z])
                rotate([0,90,0]) cylinder(d=11, h=1.4, center=true);
        }

        translate([-arm_in_x - t + 2.0, 0, yoke_axis_z])
            hole_x(d=6.5, h=4.6);

        translate([-arm_in_x - t/2 - 0.6, 0, yoke_axis_z])
            hole_x(d=3.4, h=t - 8.0);

        translate([-arm_in_x - 1.6, 0, yoke_axis_z])
            hole_x(d=2.8, h=6.6);
    }
}

module yoke() {
    axis = yoke_axis_z;
    plate_x0 = arm_in_x;
    plate_t  = 4.4;
    plate_x1 = plate_x0 + plate_t;
    body_x0  = plate_x1 + 0.25;
    cowl_x1  = body_x0 + sv_body_d + 2.2;
    difference() {
        union() {

            cylinder(d=42, h=3);

            hull() {
                translate([0,0,2.6]) cylinder(d=28, h=0.6);
                translate([0,0,5.4]) rounded_box([54, 28, 1.2], r=9);
            }

            translate([0,0,6.0]) rounded_box([54, 28, 2.4], r=9);

            difference() {
                hull() {
                    translate([(plate_x0+plate_x1)/2, 0, (axis + 17 + 5)/2])
                        soft_box([plate_t, 17.6, axis + 17 - 5], r=2.2);
                    translate([(plate_x0 + cowl_x1 + 1.6)/2, 0, side_pod_zc])
                        soft_box([cowl_x1 - plate_x0 + 1.6, side_pod_y, side_pod_h], r=side_pod_r);
                }

                translate([(plate_x1 + cowl_x1 - 2.0)/2 + 0.05, 0, 36.25])
                    cube([cowl_x1 - 2.0 - plate_x1 + 0.3, sv_body_w + 1.8, 42.0], center=true);

                translate([(body_x0 + cowl_x1 - 2.0)/2, 0, 10])
                    cube([cowl_x1 - 2.0 - body_x0 + 0.2, sv_body_w + 1.6, 13.5], center=true);

                translate([cowl_x1, 0, 36.0])
                    rotate([0,90,0])
                        rounded_box([45.0, 7.0, 10.0], r=3.4);
            }

            left_minimal_arm();
        }

        translate([0,0,1.3 - eps]) {
            cylinder(d=9.2, h=2.7, center=true);
            rounded_box([36.5, 7.4, 2.7], r=2);
        }

        hole_z(d=3.0, h=18);
        for (xx = [-7.5, 7.5])
            translate([xx,0,0]) hole_z(d=2.0, h=16);

        translate([0, 8.0, 4]) rounded_box([23, 8.0, 12], r=2.5);

        translate([0, 12.3, 7.4]) rounded_box([47, 5.8, 2.8], r=1.2);

        translate([(plate_x0+plate_x1)/2, 0, axis])
            hull() {
                hole_x(d=sv_boss_d, h=plate_t + 10);
                translate([0,0,5.5]) hole_x(d=6.0, h=plate_t + 10);
            }

        for (zz = [-sv_screw_sp/2, sv_screw_sp/2])
            translate([(plate_x0+plate_x1)/2, 0, axis + zz])
                hole_x(d=1.9, h=plate_t + 8);

        for (xx = [26, 30])
            translate([xx, 7.5, 11])
                rounded_box([1.8, 6, 7], r=0.8);
    }
}

module camera_head() {
    difference() {

        soft_box([head_w, head_d, head_h], r=5.5);

        translate([0, 1.3, 0])
            soft_box([head_w - 2.6, head_d + 0.2, head_h - 2.6], r=3.0);

        translate([0, 5.6, 0])
            rounded_box([27.5, 12.0, 26.5], r=4);

        translate([0, -head_d/2, 0]) hole_y(d=lens_d, h=head_d + 6);
        translate([0, -head_d/2 - 0.4, 0])
            rotate([90,0,0])
                cylinder(d1=lens_d, d2=lens_d + 3.5, h=1.8, center=true);

        for (x = [-cam_screw_sp_x/2, cam_screw_sp_x/2])
        for (z = [-cam_screw_sp_z/2, cam_screw_sp_z/2])
            translate([x, -head_d/2 + 0.7, z + cam_hole_z_off])
                hole_y(d=cam_screw_d, h=4);

        translate([0, 1, -head_h/2 + 3.2])
            rounded_box([fpc_w, head_d + 6, 6.5], r=3);
        translate([0, head_d/2 - 1, -9])
            rotate([0,90,0])
                rounded_box([13, 8, fpc_w], r=4);

        translate([-head_w/2, 0, 0]) hole_x(d=4.2, h=12);

        translate([head_w/2, 0, 0]) {
            hole_x(d=3.2, h=12);
            for (zz = [-7.5, 7.5])
                translate([0, 0, zz]) hole_x(d=2.0, h=12);
        }
    }
}


module camera_faceplate_insert() {
    fp_y = -head_d/2 - 0.68;
    fp_t = 1.4;
    fp_w = head_w - 4.0;
    fp_z = head_h - 4.0;
    fp_r = 5.4;
    faceplate_screw_access_d = cam_screw_d + 0.45;
    faceplate_screw_bezel_d = 4.5;

    difference() {
        translate([0, fp_y, 0])
            rounded_plate_xz(w=fp_w, z=fp_z, t=fp_t, r=fp_r);

        translate([0, fp_y, 0])
            hole_y(d=lens_d + 1.4, h=fp_t + 3);

        for (x = [-cam_screw_sp_x/2, cam_screw_sp_x/2])
        for (z = [-cam_screw_sp_z/2, cam_screw_sp_z/2]) {
            translate([x, fp_y, z + cam_hole_z_off])
                hole_y(d=faceplate_screw_access_d, h=fp_t + 3);

            translate([x, fp_y - fp_t/2 - 0.03, z + cam_hole_z_off])
                hole_y(d=faceplate_screw_bezel_d, h=0.35);
        }
    }
}

module assembly() {
    color("whitesmoke") base();
    color("whitesmoke") base_cover();
    color("whitesmoke") translate([0,0,base_h + 0.02]) yoke();
    color("white") translate([0,0,base_h + yoke_axis_z]) camera_head();
    color([0.02,0.02,0.02]) translate([0,0,base_h + yoke_axis_z]) camera_faceplate_insert();
}

if (show_part == "assembly") assembly();
if (show_part == "base") base();
if (show_part == "base_cover") base_cover();
if (show_part == "yoke") yoke();
if (show_part == "camera_head") camera_head();
if (show_part == "faceplate_insert") camera_faceplate_insert();
