#include "rml_gl_light.h"

void RGLLight::_init(const RGLLight *pGlLight)
{
    if (pGlLight)
    {
        this->enabled = pGlLight->enabled;
        this->lightNum = pGlLight->lightNum;
        this->ambient = pGlLight->ambient;
        this->diffuse = pGlLight->diffuse;
        this->specular = pGlLight->specular;
        this->direction = pGlLight->direction;
        this->position = pGlLight->position;
    }
}

RGLLight::RGLLight()
{
    this->setDefault();
    this->_init();
}

RGLLight::RGLLight(const RGLLight &glLight)
{
    this->_init(&glLight);
}

RGLLight::~RGLLight()
{
}

RGLLight &RGLLight::operator =(const RGLLight &glLight)
{
    this->_init(&glLight);
    return (*this);
}

bool RGLLight::getEnabled() const
{
    return this->enabled;
}

void RGLLight::setEnabled(bool enabled)
{
    this->enabled = enabled;
}

const QColor &RGLLight::getAmbient() const
{
    return this->ambient;
}

void RGLLight::setAmbient(const QColor &ambient)
{
    this->ambient = ambient;
}

const QColor &RGLLight::getDiffuse() const
{
    return this->diffuse;
}

void RGLLight::setDiffuse(const QColor &diffuse)
{
    this->diffuse = diffuse;
}

const QColor &RGLLight::getSpecular() const
{
    return this->specular;
}

void RGLLight::setSpecular(const QColor &specular)
{
    this->specular = specular;
}

const RR3Vector &RGLLight::getDirection() const
{
    return this->direction;
}

void RGLLight::setDirection(const RR3Vector &direction)
{
    this->direction = direction;
}

const RR3Vector &RGLLight::getPosition() const
{
    return this->position;
}

void RGLLight::setPosition(const RR3Vector &position)
{
    this->position = position;
}

int RGLLight::getLightNumber() const
{
    return this->lightNum;
}

void RGLLight::setLightNumber(int lightNum)
{
    this->lightNum = lightNum;
}

void RGLLight::setDefault()
{
    this->enabled = true;
//    this->lightNum = GL_LIGHT0;
    this->lightNum = 0;
    this->ambient.fromRgbF(0.1f,0.1f,0.1f,1.0f);
//    this->diffuse.fromRgbF(0.5f,0.5f,0.5f,1.0f);
    this->diffuse.fromRgbF(1.0f,1.0f,1.0f,1.0f);
    this->specular.fromRgbF(1.0f,1.0f,1.0f,1.0f);
    this->direction[0] = -0.6666667;
    this->direction[1] = 0.6666667;
    this->direction[2] = 0.3333333;
    this->position[0] = 1.0;
    this->position[1] = 1.0;
    this->position[2] = 10.0;
}
