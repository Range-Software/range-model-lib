#ifndef RML_FILE_UTILS_H
#define RML_FILE_UTILS_H

#include <QVector>
#include <QString>

#include <rbl_version.h>

namespace RFileUtils
{

    //! Remove file.
    void remove ( const QString &fileName );

    //! Rename file.
    void rename ( const QString &srcFileName,
                         const QString &dstFileName );

    //! List directory content.
    //! Directory path must contain directory separator at the end.
    unsigned int listDirectory ( const QString    &directory,
                                        QVector<QString> &directories,
                                        QVector<QString> &files);

    //! Return file extension.
    QString getExtension ( const QString &fileName );

    //! Return file name without an extension.
    QString removeExtension ( const QString &fileName );

    //! Build path.
    QString buildPath ( const QString &dirName, const QString &fileName );

    //! Create file name containing time step.
    QString getFileNameWithTimeStep ( const QString &fileName,
                                             unsigned int   timeStepNumber,
                                             unsigned int   numberWidth = 5 );

    //! Create file name containing time step.
    QString getFileNameWithOutTimeStep ( const QString &fileName,
                                                unsigned int   numberWidth = 5 );

    //! Create file name containing suffix (basename-suffix.extension).
    QString getFileNameWithSuffix ( const QString &fileName,
                                           const QString &suffix );

    //! Return file suffix.
    QString getSuffix ( const QString &fileName );

    //! Check if file exists.
    bool fileExists ( const QString &fileName );

    //! Find last file
    QString findLastFile ( const QString &filePattern );

    //! Find files
    QStringList findFiles ( const QString &filePattern );

    //! Write link file.
    void writeLink(const QString &linkFileName, const QString &targetFileName, const RVersion &version, bool binary = true);

    //! Find link target file name.
    QString findLinkTargetFileName(const QString &linkFileName, const QString &targetFileName);

};

#endif /* RML_FILE_UTILS_H */
