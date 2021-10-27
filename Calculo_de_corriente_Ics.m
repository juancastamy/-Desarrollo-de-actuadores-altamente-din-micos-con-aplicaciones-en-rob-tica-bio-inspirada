syms  VB Ics VA V0
R1 = 1.5*1000;
R2 = 10*1000;
C = 2 * 10 ^(-6);
eq1 = VA/R1 + (VA-VB)/R2 == Ics;
VA = solve(eq1,VA)
eq2 = (VA-VB)/R2 == VB/C;
Ics1 = solve(eq2,Ics)


%{
eq1 = VA/R1 + (VA-VB)/R2 == Ics;

VA = solve(eq1,VA);

VB =VA*((C*R2)/(C+R2));

I1 = (VA-VB)/10000;

eq2 = -(-I1+Ics)*R1 + R2 * I1 + I1 * C == 0 ;

ICS = simplify(solve(eq2,Ics))
%}