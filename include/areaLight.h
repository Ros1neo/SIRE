#ifndef SIRE_AREALIGHT_H
#define SIRE_AREALIGHT_H

#include "light.h"

class AreaLight : public Light
{
public:
    AreaLight(const PropertyList &propList);

    /// Average intensity (considering the area light as a point light)
    virtual Color3f intensity(const Point3f& x) const;

    /// Intensity from point \param x at the world space position \param y on the area light
    Color3f intensity(const Point3f& x, const Point3f& y) const;

    void loadTexture(const std::string& filename);

    //------------------------------------------------------------
    // Frame setters and getters
    /// sets the position of the camera
    void setPosition(const Point3f& pos) { m_position = pos; }
    /// \returns the position of the camera
    inline const Point3f& position() const { return m_position; }
    /// sets the orientation of the light
    void setOrientation(const Eigen::Quaternionf& q) {
        Eigen::Matrix4f H = Eigen::Matrix4f::Identity();
        H.topLeftCorner<3,3>() = q.toRotationMatrix();
        m_frame = Transform(H);  }
    /// \returns the light direction, i.e., the -z axis of the frame
    Vector3f direction() const { return -m_frame.getMatrix().col(2).head<3>(); }
    Vector3f direction(const Point3f& x, float* dist) const;
    /// \returns the first tangent axis of the light plane
    Vector3f uVec() const { return m_frame.getMatrix().col(1).head<3>(); }
    /// \returns the second tangent axis of the light plane
    Vector3f vVec() const { return m_frame.getMatrix().col(0).head<3>(); }
    //------------------------------------------------------------

    float size() const { return m_size; }

    void draw();

    /// Return a human-readable summary
    std::string toString() const;

protected:
    Point3f m_position;
    Transform m_frame;
    float m_size;
    Bitmap* m_texture = nullptr;
};

#endif // AREALIGHT_H

