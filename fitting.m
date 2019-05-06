%% Esempio 1-Interpolazione polinomiale di Lagrange
x=1:6;
y=rand(1,6);
figure(1);
nodi=plot(x,y,'o');
hold on;
t=linspace(1,6);
p=polyfit(x,y,5);
f=polyval(p,t);
p1=plot(t,f,'r');
title("Esempio di funzione interpolante");
legend([nodi p1],{'Nodi','Interpolante Grado 5'});

%% Esempio 2-Fenomeno di Runge
f = @(x) 1./(1 + 25*x.^2);
figure(2);
x = linspace(-1,1,500);
y_true = f(x);
fun=plot(x,y_true,'r','linewidth',2);
hold on;

N = 5; %Grado del polinomio
xdata = linspace(-1,1,N+1);
ydata = f(xdata);
p = polyfit(xdata,ydata,N);
y_fit = polyval(p,x);
poly_5 = plot(x,y_fit,'m','linewidth',2);
plot(xdata,ydata,'k.','markersize',10);

N = 15; %Grado del polinomio
xdata = linspace(-1,1,N+1);
ydata = f(xdata);
p = polyfit(xdata,ydata,N);
y_fit = polyval(p,x);
poly_15 = plot(x,y_fit,'b','linewidth',2);
plot(xdata,ydata,'k.','markersize',10);

N = 30;   %Grado del polinomio
xdata = linspace(-1,1,N+1);
ydata = f(xdata);
p = polyfit(xdata,ydata,N);
y_fit = polyval(p,x);
poly_30 = plot(x,y_fit,'g','linewidth',2);
plot(xdata,ydata,'k.','markersize',10);

axis([-1 1 -0.5 1.5]);
title("Fenomeno di Runge per polinomi interpolanti di vari gradi");
legend([fun,poly_5, poly_15,poly_30],{'Funzione','N = 5','N = 15','N = 30'},'Location','north');
%% Esempio 3-Fenomeno Runge Polinomio interpolante a tratti lineare
f = @(x) 1./(1 + 25*x.^2);
figure(3);
t = linspace(-1,1,500);
y_true = f(t);
fun=plot(t,y_true,'r','linewidth',2);
hold on;
%N=5
x = linspace(-1,1,5); 
y = f(x);
pt=interp1(x,y,t);
lin5=plot(t,pt,'k','linewidth',2);
%N=15
x1 = linspace(-1,1,15);
y1 = f(x1);
pt1=interp1(x1,y1,t);
lin15=plot(t,pt1,'m','linewidth',2);
%N=30
x2 = linspace(-1,1,30);
y2 = f(x2);
pt2=interp1(x2,y2,t);
lin30=plot(t,pt2,'g','linewidth',2);
title("Fenomeno di Runge per interpolazioni lineare a tratti");
axis([-1 1 -0.5 1.5]);
legend([fun,lin5, lin15,lin30],{'Funzione','N = 5','N = 15','N = 30'},'Location','northeast');
%Confronto con polinomio interpolante

%% Esempio 4- Confronto tra polinomio interpolante e spline naturale cubica
x=1:10;
y=rand(1,10);
figure(1);
punti=plot(x,y,'o');
hold on;

pp=csape(x,y,'second');%Spline naturale cubica
t=1:.1:10;
ys=ppval(pp,t);
spline=plot(t,ys,'g');

p=polyfit(x,y,5); %Polinomio grado 5
pl=polyval(p,t);
plot(t,pl,'y');

