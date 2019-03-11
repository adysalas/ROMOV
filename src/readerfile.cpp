#include <iostream>
#include <fstream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <visualization_msgs/Marker.h>
#include <sstream>
#include <vector>
#include <cmath>
#include <ros/console.h>


void lecturaF1(void);
void lecturaF2(void);

using namespace std;

#define radio 0.0625000l 
#define dist_rodas 0.415000l //Distancia entre el eje de las ruedas
#define CPR (4.00000*2500.00000) //contador de ticks con cuadratura
#define overflow 32768.00000l
#define overflow2 65536.00000l
#define PI 3.141617l
#define ladoCuadrado 2.000000l
#define cl 0.61612l
#define cr 6.49224l

vector<long int> encoderDerechoF1,encoderIzquierdoF1;
vector<long int> d1dF1,d1iF1,d2dF1,d2iF1;
vector<long double> velocidadDF1,velocidadIF1,anguloGiroF1,deltaVelocidadF1;
long int numberF1;
long double xAuxF1,yAuxF1,angAuxF1;
vector<long double> xF1,yF1,anguloF1;
char bufferF1[100];

vector<long int> encoderDerechoF2,encoderIzquierdoF2;
vector<long int> d1dF2,d1iF2,d2dF2,d2iF2;
vector<long double> velocidadDF2,velocidadIF2,anguloGiroF2,deltaVelocidadF2;
long int contadorAux = 1;
long int numberF2;
long double xAuxF2,yAuxF2,angAuxF2;
vector<long double> xF2,yF2,anguloF2;
char bufferF2[100];


int main(int argc, char **argv)
{

    
    if (argc == 1) {
       // ROS_INFO("%.6f", mensaje_tipo_float.data);
       ROS_INFO("PRECISA 2 ARCHIVOS");
    }
    else if(argc == 2)
    {
        ROS_INFO("TIENEN QUE SER 2. Solo se recivio %s", argv[1]);
    }
    else
    {
    
        sprintf(bufferF1,"/home/ady/catkin_ws/src/romov_homodetry/src/%s",argv[1]);
        sprintf(bufferF2,"/home/ady/catkin_ws/src/romov_homodetry/src/%s",argv[2]);

        ros::init(argc, argv, "points_and_lines");
        ros::NodeHandle n;
        ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);

        ros::Rate r(30);

        ifstream logfile(bufferF1,ios::in);
        
        if (logfile.is_open())
        {
            while (logfile >> numberF1)
            {
                if(contadorAux == 1)
                {
                    encoderDerechoF1.push_back(numberF1);
                }
                if(contadorAux == 2)
                {
                    encoderIzquierdoF1.push_back(numberF1);
                }
                if(contadorAux == 3)
                {
                    //checksumValue.push_back(numberF1);
                    contadorAux=0;
                }

                contadorAux++;
            }
            logfile.close();
        }
        
        ifstream logfile2(bufferF2,ios::in);
        contadorAux = 1;
        if (logfile2.is_open())
        {
            while (logfile2 >> numberF2)
            {
                if(contadorAux == 1)
                {
                    encoderDerechoF2.push_back(numberF2);
                   // cout << numberF2 << "\n";
                }
                if(contadorAux == 2)
                {
                    encoderIzquierdoF2.push_back(numberF2);
                }
                if(contadorAux == 3)
                {
                    //checksumValue.push_back(numberF2);
                    contadorAux=0;
                }

                contadorAux++;
            }
            logfile2.close();
        }
         ROS_INFO("ARCHIVO 1");
        lecturaF1();
       ROS_INFO("ARCHIVO 2");
        lecturaF2();

        //cout << suma_ << "\n";
        float f = 0.0;
        while (ros::ok())
        {
            visualization_msgs::Marker line_strip;
            visualization_msgs::Marker line_strip2;
            visualization_msgs::Marker gravityCenter;
            visualization_msgs::Marker gravityCenter2;

            line_strip.header.frame_id    = "/waypoints";
            line_strip.header.stamp       = ros::Time::now();
            line_strip.ns                 = "points_and_lines";
            line_strip.action             = visualization_msgs::Marker::ADD;
            line_strip.pose.orientation.w = 1.0;

            line_strip.id                 = 1;
            line_strip.type               = visualization_msgs::Marker::LINE_STRIP;
            line_strip.scale.x            = 0.1;

            line_strip.color.b            = 1.0;
            line_strip.color.a            = 1.0;

            line_strip2.header.frame_id    = "/waypoints";
            line_strip2.header.stamp       = ros::Time::now();
            line_strip2.ns                 = "points_and_lines1";
            line_strip2.action             = visualization_msgs::Marker::ADD;
            line_strip2.pose.orientation.w = 1.0;

            line_strip2.id                 = 1;
            line_strip2.type               = visualization_msgs::Marker::LINE_STRIP;
            line_strip2.scale.x            = 0.1;

            line_strip2.color.r            = 1.0;
            line_strip2.color.a            = 1.0;
            
            gravityCenter.header.frame_id    = "/waypoints";
            gravityCenter.header.stamp       = ros::Time::now();
            gravityCenter.ns                 = "points_and_lines2";
            gravityCenter.action             = visualization_msgs::Marker::ADD;
            gravityCenter.pose.orientation.x = 0;
            gravityCenter.pose.orientation.y = 0;
            gravityCenter.pose.orientation.z = 0;
            gravityCenter.pose.orientation.w = 1.0;

            gravityCenter.id                 = 1;
            gravityCenter.type               = visualization_msgs::Marker::CUBE;
            gravityCenter.scale.x            = 0.2;
            gravityCenter.scale.y            = -0.2;
            gravityCenter.scale.z            = 1;

            gravityCenter.color.r            = 1.0;
            gravityCenter.color.b            = 1.0;
            gravityCenter.color.a            = 1.0;
            gravityCenter.lifetime           = ros::Duration();

            gravityCenter2.header.frame_id    = "/waypoints";
            gravityCenter2.header.stamp       = ros::Time::now();
            gravityCenter2.ns                 = "points_and_lines3";
            gravityCenter2.action             = visualization_msgs::Marker::ADD;
            gravityCenter2.pose.orientation.x = 0;
            gravityCenter2.pose.orientation.y = 0;
            gravityCenter2.pose.orientation.z = 1;
            gravityCenter2.pose.orientation.w = 1.0;

            gravityCenter2.id                 = 1;
            gravityCenter2.type               = visualization_msgs::Marker::CUBE;
            gravityCenter2.scale.x            = 0.2;
            gravityCenter2.scale.y            = 0.2;
            gravityCenter2.scale.z            = 1;

            gravityCenter2.color.r            = 1.0;
            gravityCenter2.color.b            = 0.5;
            gravityCenter2.color.a            = 1.0;
            gravityCenter2.lifetime           = ros::Duration();


            int xauxG=0,yauxG=0,ng=0;
            int xauxG2=0,yauxG2=0;
            double alpha=0,beta=0;
            geometry_msgs::Point pg;
            double RadioTrajectoria =0;
            double ErrorDiametro =0;
            double cl_,cr_,eb_,dist_rodas_;

            for (uint32_t i = 0; i < xF1.size(); ++i)
            {
                
                geometry_msgs::Point p;

                p.x = xF1[i];
                p.y = yF1[i];
                p.z = 0;
                xauxG+=xF1[i];
                yauxG+=yF1[i];
                ng++;
                line_strip.points.push_back(p);
                ROS_INFO("%i %i %i",xauxG,yauxG,ng);
            }
            pg.x = xauxG/ng;
            pg.y = yauxG/ng;
            pg.z = 0;
            
            ng=0;
            gravityCenter.points.push_back(pg); 
            
            //cout << pg.x << " " << pg.y << " \n";
            
            for (uint32_t i = 0; i < xF2.size(); ++i)
            {
                geometry_msgs::Point q;
                q.x = xF2[i];
                q.y = yF2[i];
                q.z = 0;
               // ROS_INFO("%.6f ",xF2[i]);
                line_strip2.points.push_back(q);
                xauxG2+=xF2[i];
                yauxG2+=yF2[i];
                ng++;
            } 
            pg.x = xauxG2/ng;
            pg.y = yauxG2/ng;
            pg.z = 0;
            
            alpha = (xauxG + xauxG2)/(-4*ladoCuadrado);
            beta  = (xauxG - xauxG2)/(-4*ladoCuadrado);
            ROS_INFO_ONCE("ALPHA= %.5f Beta=%.5f",alpha,beta);
            RadioTrajectoria = (ladoCuadrado/2)/(sin(beta/2));
            ErrorDiametro = ((RadioTrajectoria + dist_rodas)/2)/((RadioTrajectoria - dist_rodas)/2);
            cl_ = 2/(ErrorDiametro + 1);
            cr_ = 2/(1/(ErrorDiametro + 1));
            eb_=(PI/2)/((PI/2)-alpha);
            dist_rodas_ = eb_ * dist_rodas;

            ROS_INFO_ONCE("CL= %.5f   CR= %.5f EB= %.5f  Dist_rodas= %.5f",cl_,cr_,eb_,dist_rodas_);

            xauxG = 0;
            yauxG = 0;
            xauxG = 0;
            yauxG = 0;
            ng=0;
            gravityCenter2.points.push_back(pg);


            marker_pub.publish(line_strip);
            marker_pub.publish(line_strip2);
            marker_pub.publish(gravityCenter);
            marker_pub.publish(gravityCenter2);
            r.sleep();
        }
 
    }
  return 0;
}


void lecturaF1(void)
{
    for (int i=0; i<encoderDerechoF1.size(); i++)
    {
        double aux_numberF1 = (encoderDerechoF1[i+1]-encoderDerechoF1[i])*(-1);
        d1dF1.push_back(aux_numberF1);
    }
    for (int i=0; i<encoderIzquierdoF1.size(); i++)
    {
        double aux_numberF1 = (encoderIzquierdoF1[i+1]-encoderIzquierdoF1[i])*(-1);
        d1iF1.push_back(aux_numberF1);
    } 
    
    for(int i=0; i<d1dF1.size(); i++)
    {
        double aux1,aux2,aux3;
        aux1 = (d1dF1[i] > overflow) ? 1 : 0;
        aux2 = (d1dF1[i] < (-1)*overflow) ? 1 : 0;
        aux3 = d1dF1[i] - ((aux1)*overflow2) + ((aux2)*overflow2);
        d2dF1.push_back(aux3);
    }
    for(int i=0; i<d1iF1.size(); i++)
    {
        double aux1,aux2,aux3;
        aux1 = (d1iF1[i] > overflow) ? 1 : 0;
        aux2 = (d1iF1[i] < (-1)*overflow) ? 1 : 0;
        aux3 = d1iF1[i] - ((aux1)*overflow2) + ((aux2)*overflow2);
        d2iF1.push_back(aux3);
        
    }
    for(int i=0; i<d2dF1.size(); i++)
    {
        double velocidad;
        velocidad = ((2.000000)*PI*radio*(d2dF1[i]/(double)CPR));//*cr; // distancia 
        velocidadDF1.push_back(velocidad);
    }
    for(int i=0; i<d2iF1.size(); i++)
    {
        double velocidad;
        velocidad = ((2.000000)*PI*radio*(d2iF1[i]/(double)CPR));//*cl;
        velocidadIF1.push_back(velocidad);
    }
    for(int i=0; i<velocidadIF1.size(); i++)
    {
        long double angulo;
        static long double angulo_ = 0;
        angulo = double((velocidadDF1[i]-velocidadIF1[i])/dist_rodas);
        angulo_ += angulo;
        anguloGiroF1.push_back(angulo_);
        //cout << angulo_ << "\n";
    }
    for(int i=0; i<velocidadIF1.size(); i++)
    {
        long double delta;
        delta = double((velocidadIF1[i]+velocidadDF1[i])/2.0000000000);
        deltaVelocidadF1.push_back(delta);
       // cout << delta << "\n";
        
    }
    xAuxF1 = 0.0000000;
    yAuxF1 = 0.0000000;
    angAuxF1 = 0.0000000;

    for(int i=1; i < deltaVelocidadF1.size(); i++)
    {
        static long double valueX = 0.0;
        static long double valueY = 0.0;
        static long double valueAngulo = 0.0;
    
        valueX = xAuxF1 + (deltaVelocidadF1[i]*(double)cos(anguloGiroF1[i]));
        xF1.push_back(valueX);
       // xAuxF1 += valueX/pow(10.0,2.0);
       xAuxF1 = valueX;

        valueY = yAuxF1 + (deltaVelocidadF1[i]*(double)sin(anguloGiroF1[i]));
        yF1.push_back(valueY);
        //yAuxF1 += valueY/pow(10.0,1.8);
        yAuxF1 = valueY;
        //cout << valueX << " " << xAuxF1 << " " << valueY << " " << yAuxF1 <<"\n";
    }  
   // ROS_INFO("ACABA ARCHIVO 1"); 
}

void lecturaF2(void)
{   
    for (int i=0; i<encoderDerechoF2.size(); i++)
    {
        double aux_numberF2 = (encoderDerechoF2[i+1]-encoderDerechoF2[i])*(-1);
        d1dF2.push_back(aux_numberF2);
    }
    for (int i=0; i<encoderIzquierdoF2.size(); i++)
    {
        double aux_numberF2 = (encoderIzquierdoF2[i+1]-encoderIzquierdoF2[i])*(-1);
        d1iF2.push_back(aux_numberF2);
    } 
    
    for(int i=0; i<d1dF2.size(); i++)
    {
        double aux1,aux2,aux3;
        aux1 = (d1dF2[i] > overflow) ? 1 : 0;
        aux2 = (d1dF2[i] < (-1)*overflow) ? 1 : 0;
        aux3 = d1dF2[i] - ((aux1)*overflow2) + ((aux2)*overflow2);
        d2dF2.push_back(aux3); 
    }
    for(int i=0; i<d1iF2.size(); i++)
    {
        double aux1,aux2,aux3;
        aux1 = (d1iF2[i] > overflow) ? 1 : 0;
        aux2 = (d1iF2[i] < (-1)*overflow) ? 1 : 0;
        aux3 = d1iF2[i] - ((aux1)*overflow2) + ((aux2)*overflow2);
        d2iF2.push_back(aux3);
        //cout << aux3 << "aux32\n";
    }
    for(int i=0; i<d2dF2.size(); i++)
    {
        double velocidad;
        velocidad = ((2.000000)*PI*radio*(d2dF2[i]/(double)CPR));//*cr; // distancia 
        velocidadDF2.push_back(velocidad);
    }
    for(int i=0; i<d2iF2.size(); i++)
    {
        double velocidad;
        velocidad = ((2.000000)*PI*radio*(d2iF2[i]/(double)CPR));//*cl;
        velocidadIF2.push_back(velocidad);
    }
    for(int i=0; i<velocidadIF2.size(); i++)
    {
        long double angulo;
        static long double angulo_ = 0;
        angulo = double((velocidadDF2[i]-velocidadIF2[i])/dist_rodas);
        angulo_ += angulo;
        anguloGiroF2.push_back(angulo_);
        //cout << angulo_ << "\n";
    }
    for(int i=0; i<velocidadIF2.size(); i++)
    {
        long double delta;
        delta = double((velocidadIF2[i]+velocidadDF2[i])/2.0000000000);
        deltaVelocidadF2.push_back(delta);
       // cout << delta << "\n";
        
    }
    xAuxF2 = 0.0000000;
    yAuxF2 = 0.0000000;
    angAuxF2 = 0.0000000;

    for(int i=1; i < deltaVelocidadF2.size(); i++)
    {
        static long double valueX = 0.0;
        static long double valueY = 0.0;
        static long double valueAngulo = 0.0;
    
        valueX = xAuxF2 + (deltaVelocidadF2[i]*(double)cos(anguloGiroF2[i]));
        xF2.push_back(valueX);
       // xAuxF1 += valueX/pow(10.0,2.0);
       xAuxF2 = valueX;

        valueY = yAuxF2 + (deltaVelocidadF2[i]*(double)sin(anguloGiroF2[i]));
        yF2.push_back(valueY);
        //yAuxF1 += valueY/pow(10.0,1.8);
        yAuxF2 = valueY;
        //cout << valueX << " " << xAuxF1 << " " << valueY << " " << yAuxF1 <<"\n";
    }  
    ROS_INFO("ACABA ARCHIVO 1"); 
}