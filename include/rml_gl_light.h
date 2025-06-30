#ifndef RML_GL_LIGHT_H
#define RML_GL_LIGHT_H

#include <QColor>

#include <rbl_r3vector.h>

class RGLLight
{

    protected:

        //! Enabled.
        bool enabled;
        //! Light num.
        int lightNum;
        //! Ambient light.
        QColor ambient;
        //! Diffuse light.
        QColor diffuse;
        //! Specular light.
        QColor specular;
        //! Direction.
        RR3Vector direction;
        //! Position.
        RR3Vector position;

    private:

        //! Internal initialization function.
        void _init (const RGLLight *pGlLight = nullptr);

    public:

        //! Constructor.
        RGLLight();

        //! Copy constructor.
        RGLLight(const RGLLight &glLight);

        //! Destructor.
        ~RGLLight();

        //! Assignment operator.
        RGLLight & operator = (const RGLLight &glLight);

        //! Return whether the light is enabled or not.
        bool getEnabled() const;

        //! Enable/disable the light.
        void setEnabled(bool enabled);

        //! Return ambient light.
        const QColor & getAmbient() const;

        //! Set ambient light.
        void setAmbient(const QColor &ambient);

        //! Return diffuse light.
        const QColor & getDiffuse() const;

        //! Set diffuse light.
        void setDiffuse(const QColor &diffuse);

        //! Return specular light.
        const QColor & getSpecular() const;

        //! Set specular light.
        void setSpecular(const QColor &specular);

        //! Return light direction.
        const RR3Vector & getDirection() const;

        //! Set light direction.
        void setDirection(const RR3Vector &direction);

        //! Return light position.
        const RR3Vector & getPosition() const;

        //! Set light position.
        void setPosition(const RR3Vector &position);

        //! Return light number.
        int getLightNumber() const;

        //! Set light number.
        void setLightNumber(int lightNum);

        //! Set default.
        void setDefault();

        //! Allow RFileIO to access private members.
        friend class RFileIO;

};

#endif /* RML_GL_LIGHT_H */
