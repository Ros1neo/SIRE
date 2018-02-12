#include "material.h"

Phong::Phong(const PropertyList &propList)
    : Diffuse(propList)
{
    m_specularColor = propList.getColor("specular",Color3f(0.9));
    m_exponent = propList.getFloat("exponent",0.2);
    m_diffuseColor = propList.getColor("diffuse",Color3f(0.2));
}

Color3f Phong::brdf(const Vector3f& viewDir, const Vector3f& dir, const Normal3f& normal, const Vector2f& uv) const
{
    Vector3f i = dir.normalized();
    Vector3f o = viewDir.normalized();
    Vector3f n = normal.normalized();

    Color3f color;

    if (o.dot(n)>Epsilon && i.dot(n)>Epsilon){
        // Diffuse
        color = Diffuse::brdf(o,i,n,uv);
        // Specular
        Vector3f reflected = -o + 2.f*(o.dot(n))*n;
        float alpha = i.dot(reflected);
        if (alpha > Epsilon)
            color +=  m_specularColor * ((m_exponent + 2.f) * std::pow(alpha, m_exponent) / (2.f* M_PI));
    }

    return color;
}

std::string Phong::toString() const
{
    return tfm::format(
        "Phong[\n"
        "  diffuse color = %s\n"
        "  specular color = %s\n"
        "  exponent = %f\n"
        "]", m_diffuseColor.toString(),
             m_specularColor.toString(),
             m_exponent);
}

REGISTER_CLASS(Phong, "phong")
