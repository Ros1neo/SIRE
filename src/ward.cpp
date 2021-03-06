#include "material.h"

Ward::Ward(const PropertyList &propList)
    : Diffuse(propList)
{
    m_specularColor = propList.getColor("specular",Color3f(0.9));
    m_alphaX = propList.getFloat("alphaX",1.0);
    m_alphaY = propList.getFloat("alphaY",1.0);
}

Color3f Ward::brdf(const Vector3f& viewDir, const Vector3f& dir, const Normal3f& normal, const Vector2f& uv) const
{
    Vector3f i = dir.normalized();
    Vector3f o = viewDir.normalized();
    Vector3f n = normal.normalized();
    Vector3f h = (i+o).normalized();
    float ax=m_alphaX,ay=m_alphaY;

    Vector3f x=(Vector3f(0,1.,0)-Vector3f(0,1.,0).dot(n)*n).normalized();
    Vector3f y=n.cross(x).normalized();

    Color3f color;
    Color3f diffuse_color = diffuseColor(uv) * M_1_PI;
    Color3f ward_color;

    float inv_pref=4.0*M_PI*ax*ay*sqrt(i.dot(n)*o.dot(n));

    if (o.dot(n)>Epsilon && i.dot(n)>Epsilon)
        ward_color=m_specularColor/inv_pref*exp(-(pow(h.dot(x)/ax,2.0)+pow(h.dot(y)/ay,2.0))/pow(h.dot(n),2.0));

    color=diffuse_color+ward_color;

    return color;
}

std::string Ward::toString() const
{
    return tfm::format(
                "Ward[\n"
                "  diffuse color = %s\n"
                "  specular color = %s\n"
                "  alphaX = %f\n"
                "  alphaY = %f\n"
                "]", m_diffuseColor.toString(),
                m_specularColor.toString(),
                m_alphaX,
                m_alphaY);
}

Vector3f Ward::sample_IS(const Vector3f inDir, const Vector3f normal, float *pdf) const{
    Vector3f i;
    int k = 0;
    do{

        double randDiffSpec= (double)rand()/RAND_MAX; // tirage aléatoire pour déterminer si le rayon provient d'une réfexion diffuse ou spéculaire
        double probDiffSpec =  m_specularColor.getLuminance() / (m_specularColor.getLuminance() + m_diffuseColor.getLuminance()); //probabilité (= proportion) qu'un rayon soit issu d'une réflextion spéculaire, par leurs luminances respectives

        Vector3f X = (Vector3f(0,1.,0)-Vector3f(0,1.,0).dot(normal)*normal).normalized(); // axes du repère
        Vector3f Y = normal.cross(X).normalized();

        double u = (double)rand()/RAND_MAX; //nombre aléatoire pour faire le tirage
        double v = (double)rand()/RAND_MAX;

        if (randDiffSpec<probDiffSpec){ //réflexion spéculaire selon la BRDF de WARD
            double phi = atan2(m_alphaY*sin(2*M_PI*v),m_alphaX*cos(2*M_PI*v));
            double theta = atan(sqrt(-log(u)/(cos(phi)*cos(phi)/m_alphaX/m_alphaX+sin(phi)*sin(phi)/m_alphaY/m_alphaY)));

            Vector3f h = (cos(phi)*sin(theta)*X+sin(phi)*sin(theta)*Y+cos(theta)*normal).normalized(); // vectur bissectrice

            double denom = 1.0f/4/M_PI/m_alphaX/m_alphaY/h.dot(inDir)/cos(theta)/cos(theta)/cos(theta);
            *pdf = denom*exp(-tan(theta)*tan(theta)*(cos(phi)*cos(phi)/m_alphaX/m_alphaX+sin(phi)*sin(phi)/m_alphaY/m_alphaY)); //calcul de la pdf
            i = (2*h.dot(inDir)*h-inDir).normalized();
        }
        else{ // réflexion diffuse (on aurait pu appeler Diffuse::Sample_IS mais pour plus de clarté je l'ai recopié)
            double phi =  2. * M_PI * u;

            double z = 1.-v;
            double sin_theta = sqrt(v*(2.-v));
            double x = cos(phi)*sin_theta;
            double y = sin(phi)*sin_theta;
            *pdf = 1.0f/2.0f/M_PI;
            i = (x*X+y*Y+z*normal).normalized();
        }
k++;
    }while (i.dot(normal)<Epsilon && k<100); //Teste si le rayon incident provient d'en dessous de la surface. Si c'est le cas, on retire un rayon et on recommence... jusqu'à ce qu'on en trouve un géométriquement convenable. Remarque : pour les surfaces convexes, les contributions provenant de rayons
    return i;
}

REGISTER_CLASS(Ward, "ward")
