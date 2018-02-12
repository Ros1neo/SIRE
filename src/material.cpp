#include "material.h"

#include <Eigen/Geometry>
#include <iostream>
#include <math.h>

void Material::loadTextureFromFile(const std::string& fileName)
{
    if (fileName.size()==0)
        std::cerr << "Material error : no texture file name provided" << std::endl;
    else
        m_texture = new Bitmap(fileName);
}

Diffuse::Diffuse(const PropertyList &propList)
{
    m_diffuseColor = propList.getColor("diffuse",Color3f(0.2));
    m_reflectivity = propList.getColor("reflectivity",Color3f(0.0));
    m_transmissivness = propList.getColor("transmissivness",Color3f(0.0));
    m_etaA = propList.getFloat("etaA",1);
    m_etaB = propList.getFloat("etaB",1);

    std::string texturePath = propList.getString("texture","");
    if(texturePath.size()>0){
        filesystem::path filepath = getFileResolver()->resolve(texturePath);
        loadTextureFromFile(filepath.str());
        setTextureScale(propList.getFloat("scale",1));
        setTextureMode(TextureMode(propList.getInteger("mode",0)));
    }
}

Color3f Diffuse::diffuseColor(const Vector2f& uv) const
{
    if(texture() == nullptr)
        return m_diffuseColor;

    float u = uv[0];
    float v = uv[1];

    // Take texture scaling into account
    u /= textureScaleU();
    v /= textureScaleV();

    // Compute pixel coordinates
    const int i = int(fabs(u - floor(u)) * texture()->cols());
    const int j = int(fabs(v - floor(v)) * texture()->rows());

    Color3f fColor = (*texture())(j,i);

    // Compute color
    switch(textureMode())
    {
    case MODULATE:
        return  fColor * m_diffuseColor;
    case REPLACE:
        return fColor;
    }
    return fColor;
}

Color3f Diffuse::brdf(const Vector3f& viewDir, const Vector3f& dir, const Normal3f& normal, const Vector2f& uv) const
{
    Vector3f i = dir.normalized();
    Vector3f o = viewDir.normalized();
    Vector3f n = normal.normalized();

    if (o.dot(n)>Epsilon && i.dot(n)>Epsilon)
        return diffuseColor(uv)/M_PI;
    return Color3f();
}

Vector3f Diffuse::sample_IS(const Vector3f inDir, const Vector3f normal, float *pdf = 0) const
{
    /*TODO TD3 2*/
    //But: tirer un échantillon aléatoire sur l'hémisphère orienté selon la normale
    double e1 = (double)rand()/RAND_MAX;
    double e2 = (double)rand()/RAND_MAX;

    double phi =  2. * M_PI * e1;
    Vector3f X = inDir.cross(normal).normalized();
    Vector3f Y = normal.cross(X).normalized();
    double z = 1.-e2;
    double sin_theta = sqrt(e2*(2.-e2));
    double x = cos(phi)*sin_theta;
    double y = sin(phi)*sin_theta;
    *pdf = 1.0f/2.0f/M_PI;
    return (+x*X+y*Y+z*normal).normalized();
}

std::string Diffuse::toString() const
{
    return tfm::format(
        "Diffuse[\n"
        "  color = %s\n"
        "]", m_diffuseColor.toString());
}

REGISTER_CLASS(Diffuse, "diffuse")
