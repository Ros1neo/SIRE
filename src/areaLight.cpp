#include "areaLight.h"
#include "material.h"
#include "glPrimitives.h"

AreaLight::AreaLight(const PropertyList &propList)
    : Light(propList.getColor("intensity", Color3f(1.f)))
{
    m_position = propList.getPoint("position", Point3f::UnitX());
    Vector3f dir = propList.getVector("direction",-Vector3f::UnitZ()).normalized();
    Eigen::Matrix3f M;
    M.col(2) = -dir;
    M.col(0) = M.col(2).unitOrthogonal();
    M.col(1) = M.col(2).cross(M.col(0));
    Eigen::Matrix4f H = Eigen::Matrix4f::Identity();
    H.topLeftCorner<3,3>() = M;
    m_frame = Transform(H);
    m_size = propList.getFloat("size",1.f);

    std::string texturePath = propList.getString("texture","");
    if(texturePath.size()>0){
        filesystem::path filepath = getFileResolver()->resolve(texturePath);
        loadTexture(filepath.str());
    }
}

Vector3f AreaLight::direction(const Point3f& x, float* dist = 0) const
{



    /*TODO TD4 1.1*/
    // But: calculer light_intensity



    return Vector3f();
}

Color3f AreaLight::intensity(const Point3f& x) const
{
    Vector3f dir = x-m_position;
    float d2 = dir.squaredNorm();
    return std::max(0.f,dir.normalized().dot(direction())) * m_intensity / d2;
}

Color3f AreaLight::intensity(const Point3f &x, const Point3f &y) const {



    /*TODO TD4 1.1*/
    // But: calculer l'intensité



    if(m_texture){



        /*TODO TD4 1.2*/
        // But: calculer l'intensité texturée



    }
    return Color3f();
}

void AreaLight::loadTexture(const std::string &filename) {
    try {
        m_texture = new Bitmap(filename);
    } catch (std::exception &e) {}
}

void AreaLight::draw()
{
    if(m_shader){
        m_shader->bind();
        Eigen::Affine3f M = Eigen::Affine3f::Identity();
        M.fromPositionOrientationScale(m_position,m_frame.getMatrix().topLeftCorner<3,3>(), Vector3f::Constant(m_size));
        glUniformMatrix4fv(m_shader->uniform("mat_obj"),  1, GL_FALSE, M.data());
        glUniform3fv(m_shader->uniform("color"), 1, (m_intensity/m_intensity.maxCoeff()).eval().data());
        Quad::draw(m_shader);
    }
}

std::string AreaLight::toString() const {
    return tfm::format(
                "AreaLight[\n"
                "  intensity = %s\n"
                "  frame = %s\n"
                "  size = %f\n"
                "]", m_intensity.toString(),
                indent(m_frame.toString(),10),
                m_size);
}

REGISTER_CLASS(AreaLight, "areaLight")
