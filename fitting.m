%% Esempio 1-Fenomeno di Runge
f = @(x) 1./(1 + 25*x.^2);
figure(1);
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
%% Esempio 2-Fenomeno Runge Polinomio interpolante a tratti lineare
f = @(x) 1./(1 + 25*x.^2);
figure(1);
t = linspace(-1,1,500);
y_true = f(t);
fun=plot(t,y_true,'r','linewidth',2);
hold on;
%N=5
x = linspace(-1,1,5); 
y = f(x);
pt=interp1(x,y,t);
lin5=plot(t,pt,'k','linewidth',2);
%N=10
x1 = linspace(-1,1,10);
y1 = f(x1);
pt1=interp1(x1,y1,t);
lin10=plot(t,pt1,'m','linewidth',2);
%N=100
x2 = linspace(-1,1,100);
y2 = f(x2);
pt2=interp1(x2,y2,t);
lin100=plot(t,pt2,'g','linewidth',2);
title("Fenomeno di Runge per interpolazioni lineare a tratti");
axis([-1 1 -0.5 1.5]);
legend([fun,lin5, lin10,lin100],{'Funzione','N = 5','N = 10','N = 100'},'Location','northeast');
%% Esempio 3- Confronto tra polinomio interpolante e spline
x=1:7;
y=rand(1,7);

ppc=csape(x,y,'second');
t=1:.1:7;
ys=ppval(ppc,t);
plot(x,y,'*',t,ys);

p=polyfit(x,y,5);
pl=polyval(p,t);
plot(x,y,'o',t,ys,'g',t,pl,'y','LineWidth',2,'MarkerSize',6);