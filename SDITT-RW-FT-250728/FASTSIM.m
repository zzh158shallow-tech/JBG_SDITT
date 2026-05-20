%% 轮轨接触FASTSIM算法
function [Fx,Fy,MU] = FASTSIM(N,f,a,b,fx,fy,fin,C11,C22,C23)

% clear
% clc
% N=73233;
% f=0.3;
% a=7.38e-3;b=3.20e-3;
% fx=2.08e-3;fy=-4.11e-5;fin=0.103;
% C11=5.48;C22=5.33;C23=3.04;

if N<=0
    Fx=0;Fy=0;Mz=0;MU=0;
else
    Nx=20;Ny=20;
    as=zeros(Ny,Nx);
    v=0.25;
    E=2.0*10^11;
    G=E/2/(1+v);
    Tz=pi/2;           %圆周率一半
    L1=8*a/3/C11/G;
    L2a=8*a/3/C22/G;
    L2b=pi*a*sqrt(a/b)/4/C23/G;
    z0=2*N/pi/a/b;           %最大接触应力
    dy=2/Ny;                 %求解划分单元格大小
    Tx=0;Ty=0;Mz=0;
    n1=a*fx/f/z0/L1;
    n2=a*fy/f/z0/L2a;
    fin1=a*b*fin/f/z0/L2b;
    fin2=a*a*fin/f/z0/L2b;
    
    for i=1:Ny
        Y=-1+i*dy-dy/2;             %网格横向坐标
        SX=n1-Y*fin1;
        px=0;py=0;
        A2=1-(Y)^2;
        X0=sqrt(A2);
        A=X0;
        h=2*A/Nx;
        AR=h*dy;
        X=A-h/2;        
        for j=1:Nx
            SY=n2+fin2*(X);
            px=px-SX*h;
            py=py-SY*h;
            Z=A2-X^2;
            p=sqrt(px^2+py^2)/Z;
            if p>1
                as(Ny-i+1,Nx-j+1)=1;
                px=px/p;
                py=py/p;
            end
            Tx=Tx+px*AR;
            Ty=Ty+py*AR;
            Mz=Mz-((px*AR)/Tz*f*N)*(Y*b)+((py*AR)/Tz*f*N)*(X*a);
            X0=X;
            X=X-h;
            sl(i,j)=p;
        end
    end
end
Fx=Tx/Tz*f*N;   %纵向蠕滑力
Fy=Ty/Tz*f*N;   %横向蠕滑力
MU=sqrt(Tx^2+Ty^2)/Tz*f;
x=linspace(-a,a,200)';
y1= b*sqrt(1-x.^2/a^2);
y2=-b*sqrt(1-x.^2/a^2);

% plot(x,y1,x,y2)
% hold on
% axis equal
% delty=2*b/Ny;
% y_0=linspace(-(b-delty/2),(b-delty/2),Ny);
% for i=1:Ny
%     x_0(i)=a*sqrt(1-y_0(i)^2/b^2);
%     deltx(i)=x_0(i)/Nx;
% end
% M_y=zeros(Ny,Nx);
% M_x=zeros(Ny,Nx);
% for i=1:1:Nx
%     M_y(:,i)=y_0;
% end
% for i=1:1:Ny
%     M_x(i,:)=linspace(-x_0(i),x_0(i),Nx);
% end

% %红色的是黏着区，蓝色的是滑动区
% for i=1:Ny
%     for j=1:Nx
%         if as(i,j)==0;
%             plot(M_x(i,j),M_y(i,j),'r*')
%         elseif as(i,j)==1;
%             plot(M_x(i,j),M_y(i,j),'b*')
%         end
%     end
% end

