syms vx_m vy_m vz_m phi_s the_s sci_s x_s y_s z_s bi_1 bi_2 bi_3 ox_m oy_m oz_m...
    nsx_v nsy_v nsz_v nsx_g nsy_g nsz_g X u n t
% _s: state _p: previous _m: measurement

G = [cos(the_s), 0, -cos(phi_s)*sin(the_s);...
    0, 1, sin(phi_s);...
    sin(the_s), 0, cos(phi_s)*cos(the_s)];

X = [x_s;
    y_s;
    z_s;
    phi_s;
    the_s;
    sci_s;
    bi_1;
    bi_2;
    bi_3];

u = [vx_m;
    vy_m;
    vz_m;
    ox_m;
    oy_m;
    oz_m];

n = [nsx_v;
    nsy_v;
    nsz_v;
    nsx_g;
    nsy_g;
    nsz_g;
    diff(bi_1,t);
    diff(bi_2,t);
    diff(bi_3,t)]

f = [vx_m - nsx_v;
    vy_m - nsy_v;
    vz_m - nsz_v;
    G \ ([ox_m - bi_1 - nsx_g;
    oy_m - bi_2 - nsy_g;
    oz_m - bi_3 - nsz_g]);
    diff(bi_1);
    diff(bi_2);
    diff(bi_3)]

f_x = [diff(f, x_s)';
    diff(f, y_s)';
    diff(f, z_s)';
    diff(f, phi_s)';
    diff(f, the_s)';
    diff(f, sci_s)';
    diff(f, bi_1)';
    diff(f, bi_2)';
    diff(f, bi_3)']

f_u = [diff(f, vx_m)';
    diff(f, vy_m)';
    diff(f, vz_m)';
    diff(f, ox_m)';
    diff(f, oy_m)';
    diff(f, oz_m)']

f_n = [diff(f, nsx_v)';
    diff(f, nsy_v)';
    diff(f, nsz_v)';
    diff(f, nsx_g)';
    diff(f, nsy_g)';
    diff(f, nsz_g)';
    diff(f, diff(bi_1))';
    diff(f, diff(bi_2))';
    diff(f, diff(bi_3))']
