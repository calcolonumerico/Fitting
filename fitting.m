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
%% Esempio 3- Interpolazione a tratti
f = @(x) sin(3*x)+cos(x/2);
figure(3);
t = linspace(0,2*pi,500);
y_true = f(t);
fun=plot(t,y_true,'r','linewidth',2);
hold on;
%N=5
x = linspace(0,2*pi,5); 
y = f(x);
pt=interp1(x,y,t);
lin5=plot(t,pt,'k','linewidth',2);
%N=15
x1 = linspace(0,2*pi,15);
y1 = f(x1);
pt1=interp1(x1,y1,t);
lin15=plot(t,pt1,'m','linewidth',2);
%N=30
x2 = linspace(0,2*pi,30);
y2 = f(x2);
pt2=interp1(x2,y2,t);
lin30=plot(t,pt2,'g','linewidth',2);
title("Esempio di interpolazione lineare a tratti");
axis([0 2*pi -5 5]);
legend([fun,lin5, lin15,lin30],{'Funzione','N = 5','N = 15','N = 30'},'Location','northeast');

%% Esempio 4-Confronto tra interpolante a tratti lineare e  cubica
f = @(x) sin(3*x)+cos(x/2);
figure(4);
t = linspace(0,2*pi,500);
y_true = f(t);
fun=plot(t,y_true,'r','linewidth',2);
hold on;
%Lineare
x = linspace(0,2*pi,30);
y = f(x);
pt=interp1(x,y,t);
lin=plot(t,pt,'g','linewidth',2);
%Cubica
ptc=interp1(x,y,t,'pchip'); 
cub=plot(t,ptc,'m','linewidth',2);
title("Confronto interpolazione lineare e cubica a tratti");
axis([1.2 2 -0.6 0.6]);
legend([fun,lin,cub],{'Funzione','Lineare','Cubica'},'Location','northeast');

%% Esempio 5- Confronto tra polinomio interpolante e spline naturale cubica
x=1:6;
y=rand(1,6);
figure(5);
punti=plot(x,y,'k.','MarkerSize',20);
hold on;

pp=csape(x,y,'second');%Spline naturale cubica
t=1:.1:6;
ys=ppval(pp,t);
spline=plot(t,ys,'r');

p=polyfit(x,y,5); %Polinomio grado 5
pl=polyval(p,t);
pol=plot(t,pl,'b');
title("Confronto polinomio interpolante e spline naturale");
legend([punti,spline,pol],{'Punti','Spline Naturale','Pol. Grado 5'},'Location','northeast');
%% Esempio 6 - Confronto pol interp spline naturale cubica ?(titanium)
[x,y]=titanium;
figure(6);
punti=plot(x,y,'k.','MarkerSize',20);
hold on;

t=linspace(595,1075);
pp=csape(x,y,'second');
ys=ppval(pp,t);
spline=plot(t,ys,'r');

p=polyfit(x,y,48); 
pl=polyval(p,t);
pol=plot(t,pl,'b');
axis([595 1075 -10 10]);
title("Confronto polinomio interpolante e spline naturale");
legend([punti,spline,pol],{'Punti','Spline Naturale','Pol. Interpolante'},'Location','northeast');
%% Esempio 7- Spline not a knot e sua derivata
%Definizione spline e derivata
x=linspace(-5,5,15);
xx=linspace(-5,5);
y=exp(-x).*cos(x);
yy=exp(-xx).*cos(xx);
yd=-exp(-xx).*(sin(xx)+cos(xx));
figure(7)
subplot(2,1,1);
punti=plot(x,y,'*');
hold on;
%pp form spline e derivata
pp=spline(x,y);
sder=fnder(pp,1);
%valutazione spline e derivata
sp=ppval(pp,xx);
sd=ppval(sder,xx);
spl=plot(xx,sp,'b');
fun=plot(xx,yy,'r');
legend([punti,spl,fun],{'Punti','Spline','Funzione'},'Location','northeast');
subplot(2,1,2);
hold on;
spld=plot(xx,sd,'b');
fund=plot(xx,yd,'r');
legend([spld,fund],{'Derivata Spline','Derivata Funzione'},'Location','southeast');
%% Esempio 8- Confronto vari tipi spline
x=linspace(-1,1,7);
y = 1./(1 + 25*x.^2);
yf=1./(1 + 25*xx.^2);
figure(8);
xx = linspace(-1,1,101);
nodi=plot(x,y,'k.','MarkerSize',20);
hold on;
fun=plot(xx,yf,'r','linewidth',2);
%not a knot
cs=spline(x,y,xx);
knot=plot(xx,cs,'b');
%naturale
pp=csape(x,y,'second');
yy=ppval(pp,xx);
spl=plot(xx,yy,'g');
legend([nodi,fun,knot,spl],{'Nodi','Funzione','Spline not a knot','Spline naturale'},'Location','northeast');
%% Interpolazione parametrica
