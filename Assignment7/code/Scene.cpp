//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    //TO DO Implement Path Tracing Algorithm here
    Intersection interP=intersect(ray);
    if(!interP.happened) //if ray hits nothing
        return this->backgroundColor;

    Vector3f hitPoint_P=interP.coords;
    Vector3f normal_P=interP.normal;
    Material* m_P=interP.m;

    Intersection interL;
    float pdf_light=0.f;
    sampleLight(interL, pdf_light);
    Vector3f hitPoint_L=interL.coords;
    Vector3f normal_L=interL.normal;
    Vector3f emit=interL.emit;

    Vector3f wi=(hitPoint_L-hitPoint_P).normalized();
    Vector3f wo=ray.direction;

    Vector3f L_dir(0.0f), L_indir(0.0f);

    auto isInShadow=[&]()->bool{
        Vector3f shadowPoint=(dotProduct(wo,normal_P)<0.f)?
                                hitPoint_P+normal_P*EPSILON:
                                hitPoint_P-normal_P*EPSILON;
        Intersection interS=intersect(Ray(shadowPoint,wi));
        if(fabs(interS.distance-(hitPoint_L-shadowPoint).norm())>=0.01f)
            return true;
        return false;
    };

    if(!isInShadow()&&pdf_light>EPSILON)
    {
        L_dir=emit*m_P->eval(wo,wi,normal_P)*dotProduct(wi,normal_P)*dotProduct(-wi,normal_L)/
              (dotProduct(hitPoint_P-hitPoint_L,hitPoint_P-hitPoint_L)*pdf_light);
    }

    if(get_random_float()<=RussianRoulette)
    {
        wi=m_P->sample(wo,normal_P).normalized();
        Vector3f shadowPoint=(dotProduct(wo,normal_P)<0.f)?
                                hitPoint_P+normal_P*EPSILON:
                                hitPoint_P-normal_P*EPSILON;
        Ray shadeRay=Ray(shadowPoint,wi);
        Intersection interNext=intersect(shadeRay);
        
        if(interNext.happened&&!interNext.m->hasEmission())
        {
            float pdf=m_P->pdf(wo,wi,normal_P);
            if(pdf>EPSILON)
            {
                L_indir=castRay(shadeRay,depth+1)*
                        m_P->eval(wo,wi,normal_P)*
                        dotProduct(wi,normal_P)/(pdf*RussianRoulette);
            }
        }
    }

    return m_P->getEmission() + L_dir + L_indir;
}