#include <QDir>
#include <QFileInfo>

#include <rbl_error.h>
#include <rbl_logger.h>

#include "rml_file_utils.h"
#include "rml_file_io.h"

namespace RFileUtils
{

    void remove(const QString &fileName)
    {
        if (!QFile::remove(fileName))
        {
            throw RError(RError::Type::RemoveFile,R_ERROR_REF,"Failed to remove the file \'" + fileName + "\'");
        }
    }

    void rename(const QString &srcFileName, const QString &dstFileName)
    {
        if (!QFile::rename(srcFileName,dstFileName))
        {
            throw RError(RError::Type::RemoveFile,R_ERROR_REF,"Failed to rename the file \'" + srcFileName + "\' to \'" + dstFileName + "\'");
        }
    }

    unsigned int listDirectory(const QString &directory,
                                             QVector<QString> &directories,
                                             QVector<QString> &files)
    {
        QDir dir(directory);

        QStringList dirList = dir.entryList(QDir::Dirs);
        QStringList fileList = dir.entryList(QDir::Files);

        directories = dirList.toVector();
        files = fileList.toVector();

        return directories.size() + files.size();
    }

    QString getExtension(const QString &fileName)
    {
        QFileInfo fi(fileName);
        return fi.suffix();
    }

    QString removeExtension(const QString &fileName)
    {
        QFileInfo fi(fileName);
        return buildPath(fi.path(),fi.baseName());
    }

    QString buildPath(const QString &dirName, const QString &fileName)
    {
        QDir dir(dirName);
        return dir.filePath(fileName);
    }

    QString getFileNameWithTimeStep(const QString &fileName, unsigned int timeStepNumber, unsigned int numberWidth)
    {
        if (fileName.isEmpty())
        {
            return QString();
        }

        QString timeStepString = QString("%1").arg(timeStepNumber, numberWidth, 10, QChar('0')).toUpper();

        QString fileNameWithTs(fileName);

        int dashPos = fileName.lastIndexOf("-");
        int dotPos = fileName.lastIndexOf(".");

        if (dotPos < 0)
        {
            dotPos = fileName.size();
        }

        if (dashPos < 0)
        {
            dashPos = dotPos;
        }

        bool replace = false;
        if (dashPos != dotPos && (dotPos-dashPos-1) == int(numberWidth))
        {
            replace = true;
            for (uint i=0;i<numberWidth;i++)
            {
                if (!fileName[i+dashPos+1].isDigit())
                {
                    replace = false;
                    break;
                }
            }
        }
        if (replace)
        {
            fileNameWithTs.replace(dashPos+1,dotPos-dashPos-1,timeStepString);
        }
        else
        {
            fileNameWithTs.insert(dotPos,QString("-").append(timeStepString));
        }

        return fileNameWithTs;
    }

    QString getFileNameWithOutTimeStep(const QString &fileName, unsigned int numberWidth)
    {
        if (fileName.isEmpty())
        {
            return QString();
        }

        int dashPos = fileName.lastIndexOf("-");
        int dotPos = fileName.lastIndexOf(".");

        if (dotPos < 0)
        {
            dotPos = fileName.size();
        }

        if (dashPos < 0)
        {
            return fileName;
        }

        if (dashPos != dotPos && (dotPos-dashPos-1) == int(numberWidth))
        {
            for (uint i=0;i<numberWidth;i++)
            {
                if (!fileName[i+dashPos+1].isDigit())
                {
                    return fileName;
                }
            }
            QString fileNameWithoutTs(fileName);

            fileNameWithoutTs.remove(dashPos,dotPos-dashPos);

            return fileNameWithoutTs;
        }
        else
        {
            return fileName;
        }
    }

    QString getFileNameWithSuffix(const QString &fileName, const QString &suffix)
    {
        return removeExtension(fileName) + "-" + suffix + "." + getExtension(fileName);
    }

    QString getSuffix(const QString &fileName)
    {
        if (fileName.isEmpty())
        {
            return QString();
        }

        int dashPos = fileName.lastIndexOf("-");
        int dotPos = fileName.lastIndexOf(".");

        if (dotPos < 0)
        {
            dotPos = fileName.size();
        }

        if (dashPos < 0)
        {
            return QString();
        }

        if (dashPos != dotPos && dotPos > dashPos)
        {
            QString suffix(fileName);

            suffix.remove(dotPos,suffix.length() - dotPos);
            suffix.remove(0,dashPos);

            return suffix;
        }
        else
        {
            return QString();
        }
    }

    bool fileExists(const QString &fileName)
    {
        return QFile::exists(fileName);
    }

    QString findLastFile(const QString &filePattern)
    {
        QFileInfo fileInfo(filePattern);

        QStringList nameFilters;
        nameFilters << fileInfo.fileName();

        QDir dir(fileInfo.absolutePath());

        QStringList fileList = dir.entryList(nameFilters,QDir::Files,QDir::Reversed);

        if (fileList.count() > 0)
        {
            return dir.filePath(fileList.at(0));
        }
        return QString();
    }

    QStringList findFiles(const QString &filePattern)
    {
        QFileInfo fileInfo(filePattern);

        QStringList nameFilters;
        nameFilters << fileInfo.fileName();

        QDir dir(fileInfo.absolutePath());

        QStringList fileList = dir.entryList(nameFilters,QDir::Files,QDir::Reversed);

        QStringList fileNames;
        for (int i=0; i<fileList.count();i++)
        {
            fileNames.append(dir.filePath(fileList.at(i)));
        }
        return fileNames;
    }

    void writeLink(const QString &linkFileName, const QString &targetFileName, const RVersion &version, bool binary)
    {
        if (linkFileName.isEmpty() || targetFileName.isEmpty())
        {
            throw RError(RError::Type::InvalidFileName,R_ERROR_REF,"No file name was provided.");
        }

    //    QString ext = RFileManager::getExtension(linkFileName);
        // Find relative path.
        QDir linkFileDir(QFileInfo(linkFileName).absoluteDir());
        QString relativeTargetFileName(linkFileDir.relativeFilePath(targetFileName));

        if (binary)
        {
            RLogger::info("Writing binary link file \'%s\'\n",linkFileName.toUtf8().constData());
            RLogger::debug("Version: \'%s\'\n",version.toString().toUtf8().constData());
            RLogger::debug("Target: \'%s\'\n",relativeTargetFileName.toUtf8().constData());

            RSaveFile linkFile(linkFileName,RSaveFile::BINARY);

            if (!linkFile.open(QIODevice::WriteOnly))
            {
                throw RError(RError::Type::OpenFile,R_ERROR_REF,"Failed to open the file \'%s\'.",linkFileName.toUtf8().constData());
            }
            RFileIO::writeBinary(linkFile,RFileHeader(R_FILE_TYPE_LINK,version,relativeTargetFileName));

            linkFile.commit();
        }
        else
        {
            RLogger::info("Writing ascii link file \'%s\'\n",linkFileName.toUtf8().constData());
            RLogger::debug("Version: \'%s\'\n",version.toString().toUtf8().constData());
            RLogger::debug("Target: \'%s\'\n",relativeTargetFileName.toUtf8().constData());

            RSaveFile linkFile(linkFileName,RSaveFile::ASCII);

            if (!linkFile.open(QIODevice::WriteOnly | QIODevice::Text))
            {
                throw RError(RError::Type::OpenFile,R_ERROR_REF,"Failed to open the file \'%s\'.",linkFileName.toUtf8().constData());
            }

            RFileIO::writeAscii(linkFile,RFileHeader(R_FILE_TYPE_LINK,version,"\"" + relativeTargetFileName + "\""));

            linkFile.commit();
        }
    }

    QString findLinkTargetFileName(const QString &linkFileName, const QString &targetFileName)
    {
        QString processedTargetFileName(targetFileName);
        if (QFileInfo(processedTargetFileName).isRelative())
        {
            processedTargetFileName = QDir::cleanPath(buildPath(QFileInfo(linkFileName).absoluteDir().absolutePath(),processedTargetFileName));
        }

        return processedTargetFileName;
    }
}
