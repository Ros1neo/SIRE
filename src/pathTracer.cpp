#include "integrator.h"
#include "scene.h"
#include "material.h"
#include "areaLight.h"

class PathTracer : public Integrator
{
public:
    PathTracer(const PropertyList &props) {
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

            // Direct lights (on a recopié l'implémentation de Whitted
            const LightList &lights = scene->lightList();
            for(LightList::const_iterator it=lights.begin(); it!=lights.end(); ++it)
            {
                float dist;
                Vector3f lightDir = (*it)->direction(pos, &dist);
                Color3f light_intensity;
                if(dynamic_cast<const AreaLight*>(*it)){
                    // source étendue
                    const AreaLight* light =  dynamic_cast<const AreaLight*>(*it);

                    /*TODO TD4 1.1*/
                    // But: calculer light_intensity
                    Ray lightRay(pos,lightDir);
                    light_intensity = light->intensity(pos,lightRay.at(dist));
                }
                else{
                    // lampe ponctuelle ou directionnelle
                    light_intensity = (*it)->intensity(pos);
                }
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
                radiance += light_intensity * cos_term * brdf * attenuation;
            }

            // Env Lights
            if (m_IS ){
                double tirage = (double)rand()/RAND_MAX; //tirage de la roulette russe
                float pAbs = material->ambientColor().getLuminance(); //probabilité de se faire absorber
                if(tirage>pAbs && pAbs<1){ //Le rayon n'est pas absorbé
                /*TODO TD3 2*/
                //But: intégrer l'équation du rendu au sens de Monte-Carlo pour m_samples échantillons
                    //On n'échantillonne plus 100 fois à chaque intersection. à la place, on tire 100 rayons sur le même point d'impact ce qui réduit le mon
                float pdfEchantillon;
                Ray Echantillon(pos + hit.normal()*1e-4,material->sample_IS(-ray.direction,normal,&pdfEchantillon));
                Echantillon.recursionLevel = ray.recursionLevel + 1;

                float cos_term = std::max(0.f,Echantillon.direction.dot(normal));

                Color3f brdf = material->brdf(-ray.direction,Echantillon.direction,normal, hit.texcoord());

                radiance+=1.0f*Li(scene,Echantillon) * cos_term * brdf / pdfEchantillon/(1.0f-pAbs);//correction par pdfechantillon et roulette russe
                }
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
        return tfm::format("PathTracer[\n"
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

REGISTER_CLASS(PathTracer, "path_mats")
