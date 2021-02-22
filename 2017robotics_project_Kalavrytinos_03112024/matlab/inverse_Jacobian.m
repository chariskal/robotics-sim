syms s1 c1 s2 c2 s23 c23 l1 l2 l3 l4 l5 l0
J=[-s1*l2+c1*l3+c1*c2*l4+c1*c23*l5, -s1*s2*l4-s1*s23*l5, -s1*s23*l5; 0, c2*l4+c23*l5, c23*l5; c1*l2+s1*l3+s1*c2*l4+s1*c23*l5, s2*c1*l4+s23*c1*l5, c1*s23*l5];

Jminus1=inv(J);