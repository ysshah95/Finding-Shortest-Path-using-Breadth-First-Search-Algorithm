function [in] = insidepoly_halfplane(c,d)

% Square 

if c <= 105 && c >= 55 && d >= 67.5 && d <= 112.5
    ins = true;
else
    ins = false;
end

% polygon

x1 = 120;
y1 = 55;

x2 = 145;
y2 = 14;

x3 = 158;
y3 = 51;

x4 = 165;
y4 = 89;

x5 = 188;
y5 = 51;

x6 = 168;
y6 = 14;

x = c;
yy1 = y1 + ((y2-y1)*(x-x1))/(x2-x1);
yy2 = y1 + ((y3-y1)*(x-x1))/(x3-x1);
yy3 = y3 + ((y2-y3)*(x-x3))/(x2-x3);
yy4 = y3 + ((y4-y3)*(x-x3))/(x4-x3);
yy5 = y4 + ((y5-y4)*(x-x4))/(x5-x4);
yy6 = y5 + ((y6-y5)*(x-x5))/(x6-x5);
yy7 = y6 + ((y2-y6)*(x-x6))/(x2-x6);
yy8 = y3 + ((y5-y3)*(x-x3))/(x5-x3);
yy9 = y3 + ((y6-y3)*(x-x3))/(x6-x3);

if (d>=yy1 && d<=yy2 && d>=yy3) | (d<=yy3 && d>=yy7 && d<=yy9) |  (d<=yy4 && d>=yy8 && d<yy5) || (d>=yy9 && d>=yy6 && d<=yy8)
    inp = true;
else
    inp = false;
end

% circle 

h = 180;
k = 120;

y = ((c-h)^2 + (d-k)^2)^(0.5);

if y <= 15
    inc = true;
else
    inc = false;
end

in = ins | inp | inc;





        
