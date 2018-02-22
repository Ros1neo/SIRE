#include "integrator.h"
#include "scene.h"
#include "material.h"

class DirectMats : public Integrator
{
public:
    DirectMats(const PropertyList &props) {
        m_maxRecursion = props.getInteger("maxRecursion",4);
        m_samples = props.getInteger("samples",100);
        m_IS = props.getBoolean("IS",false);
    }

    Color3f Li(const Scene *scene, const Ray &ray) const {

        Color3f radiance = Color3f::Zero();

        // stopping criteria:
        if(ray.recursionLevel>=m_maxRecursion) {
            return radiance;
        }

        /* Find the surface that is visible in the requested direction */
        Hit hit;
        scene->intersect(ray, hit);
        if (hit.foundIntersection())
        {
            Point3f pos = ray.at(hit.t());
            Normal3f normal = hit.normal();

            const Material* material = hit.shape()->material();

            // Direct Lights
            const LightList &lights = scene->lightList();
            for(LightList::const_iterator it=lights.begin(); it!=lights.end(); ++it)
            {
                float dist;
                Vector3f lightDir = (*it)->direction(pos, &dist);
                Ray shadowRay(pos + normal*1e-4, lightDir,true);
                Hit shadowHit;
                scene->intersect(shadowRay,shadowHit);
                Color3f attenuation = Color3f(1.f);
                if(shadowHit.t()<dist){
                    attenuation = 0.5f * shadowHit.shape()->material()->transmissivness();
                    if((attenuation <= 1e-6).all())
                        continue;
                }

                float cos_term = std::max(0.f,lightDir.dot(normal));
                Color3f brdf = material->brdf(-ray.direction, lightDir, normal, hit.texcoord());
                radiance += (*it)->intensity(pos) * cos_term * brdf * attenuation;
            }

            // Env Lights
            if (m_IS){
                /*TODO TD3 2*/
                //But: intégrer l'équation du rendu au sens de Monte-Carlo pour m_samples échantillons
                for(int i=0; i<m_samples; i++){
                    float pdfEchantillon;
                    Ray Echantillon(pos + hit.normal()*1e-4,material->sample_IS(-ray.direction,normal,&pdfEchantillon)); // Rayon de stockage des coordonnées de Echantillon, le Rayon incident
                    Echantillon.recursionLevel = ray.recursionLevel + 1; // sert à avoir une limite au nobre d'iterations
                    float cos_term = std::max(0.f,Echantillon.direction.dot(normal)); // terme cosinus
                    Color3f brdf = material->brdf(-ray.direction,Echantillon.direction,normal, hit.texcoord()); //BRDF entre le rayon incident et le rayon de sortie (= celui dont on part)
                    radiance+=1.0f/m_samples*Li(scene,Echantillon) * cos_term * brdf / pdfEchantillon;}  //Equation du rendu, avec appel résursif de Li pour remonter aux sources/background
            }

            // Reflexions
            if((material->reflectivity() > 1e-6).any())
            {
                Vector3f r = (ray.direction - 2.*ray.direction.dot(hit.normal())*hit.normal()).normalized();
                Ray reflexion_ray(pos + hit.normal()*1e-4, r);
                reflexion_ray.recursionLevel = ray.recursionLevel + 1;
                float cos_term = std::max(0.f,r.dot(normal));
                radiance += material->reflectivity() * Li(scene, reflexion_ray) * cos_term;
            }

        }else
            return scene->backgroundColor(ray.direction);

        return radiance;
    }

    std::string toString() const {
        return tfm::format("DirectMats[\n"
                           "  max recursion = %f\n"
                           "  samples = %d\n"
                           "  IS = %b\n"
                           " ]\n",
                           m_maxRecursion,
                           m_samples,
                           m_IS);
    }
private:
    int m_maxRecursion;
    int m_samples;
    bool m_IS;
};

REGISTER_CLASS(DirectMats, "direct_mats")
