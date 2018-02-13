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
    do{
    double u = (double)rand()/RAND_MAX;
    double v = (double)rand()/RAND_MAX;

    double phi = atan2(m_alphaY*sin(2*M_PI*v),m_alphaX*cos(2*M_PI*v));
    double theta = atan(sqrt(-log(u)/(cos(phi)*cos(phi)/m_alphaX/m_alphaX+sin(phi)*sin(phi)/m_alphaY/m_alphaY)));

    Vector3f X = (Vector3f(0,1.,0)-Vector3f(0,1.,0).dot(normal)*normal).normalized();
    Vector3f Y = normal.cross(X).normalized();

    Vector3f h = (cos(phi)*sin(theta)*X+sin(phi)*sin(theta)*Y+cos(theta)*normal).normalized();

    double denom = 1.0f/4/M_PI/m_alphaX/m_alphaY/h.dot(inDir)/cos(theta)/cos(theta)/cos(theta);
    *pdf = denom*exp(-tan(theta)*tan(theta)*(cos(phi)*cos(phi)/m_alphaX/m_alphaX+sin(phi)*sin(phi)/m_alphaY/m_alphaY));
    i = (2*h.dot(inDir)*h-inDir).normalized();

    }while (i.dot(normal)<0);
    return i;
}

REGISTER_CLASS(Ward, "ward")
