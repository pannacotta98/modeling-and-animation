#pragma once

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb_image_write.h>

template <typename DataType>
Image<DataType>::Image(const std::string &filename) : Image() {
    // Try to read the file
    this->ReadFromFile(filename);
}

template <typename DataType>
Image<DataType>::Image(const Image &img) : Image(img.mWidth, img.mHeight, img.mComponents) {
    std::copy(std::begin(img.mData), std::end(img.mData), std::begin(mData));
}

template <typename DataType>
void Image<DataType>::SetDimensions(GLuint width, GLuint height, GLuint components) {
    mWidth = width;
    mHeight = height;
    mComponents = components;

    mData = std::vector<DataType>(mWidth * mHeight * mComponents);
}

template <typename DataType>
void Image<DataType>::ReadFromFile(const std::string &filename) {
    GLubyte TGAcomment[1];
    GLubyte TGAheader[] = {0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    GLubyte TGAcompare[11];
    GLubyte header[6];
    int imageSize;

    FILE *file = fopen(filename.c_str(), "rb");

    if (file == NULL || fread(TGAcomment, 1, sizeof(TGAcomment), file) != sizeof(TGAcomment) ||
        fread(TGAcompare, 1, sizeof(TGAcompare), file) != sizeof(TGAcompare) ||
        memcmp(TGAheader, TGAcompare, sizeof(TGAheader)) != 0 ||
        fread(header, 1, sizeof(header), file) != sizeof(header)) {
        fprintf(stderr, "\nTGA load error.\n");
        fclose(file);
        return;
    }

    char *comment = new char[TGAcomment[0]];
    if (TGAcomment[0] != 0) {
        fread(comment, 1, sizeof(comment), file);
        printf("TGA comment: %s\n", comment);
    }
    delete[] comment;

    this->SetDimensions(header[1] * 256 + header[0], header[3] * 256 + header[2], header[4] / 8);

    if (mWidth <= 0 || mHeight <= 0 || mComponents < 1 || mComponents > 4) {
        fprintf(stderr, "\nTGA load error.\n");
        fclose(file);
        return;
    }

    imageSize = mWidth * mHeight * mComponents;

    std::vector<GLubyte> data(imageSize);

    if ((int)fread(data.data(), 1, imageSize, file) != imageSize) {
        data.clear();
        fprintf(stderr, "\nTGA load error.\n");
        fclose(file);
        return;
    }

    // Copy data to member storage buffer
    // Remember to normalize values
    std::transform(std::begin(data), std::end(data), std::begin(mData),
                   [](const auto val) { return static_cast<DataType>(val) / DataType{255}; });

    // Flip red and blue components
    if (mComponents > 2) {
        for (int i = 0; i < imageSize; i += mComponents) {
            std::swap(mData[i], mData[i + 2]);
        }
    }

    fclose(file);
    printf("TGA file '%s' opened [%d x %d, %i Bpp]\n\n", filename.c_str(), mWidth, mHeight,
           mComponents);
}

template <typename DataType>
void Image<DataType>::SaveToFile(const std::string &filename) {
    std::vector<GLubyte> data(mData.size());

    std::transform(std::begin(mData), std::end(mData), std::begin(data),
                   [](const auto val) { return static_cast<GLubyte>(val * DataType{255}); });

    stbi_flip_vertically_on_write(1);

    if (!stbi_write_png(filename.c_str(), mWidth, mHeight, mComponents, data.data(),
                        static_cast<int>(mWidth * mComponents * sizeof(GLubyte)))) {
        std::cerr << "Could not write PNG file." << std::endl;
        return;
    }

    std::cout << "PNG file written to '" << filename << "'" << std::endl << std::endl;
}

template <typename DataType>
void Image<DataType>::CaptureScreen() {
    // Get size of the window
    GLint viewport[4];
    glGetIntegerv(GL_VIEWPORT, viewport);

    this->SetDimensions(viewport[2], viewport[3], 4);

    glFinish();

    std::vector<GLubyte> data(viewport[2] * viewport[3] * 4);

    glReadPixels(0, 0, mWidth, mHeight, GL_RGBA, GL_UNSIGNED_BYTE, data.data());

    std::transform(std::begin(data), std::end(data), std::begin(mData),
                   [](const auto val) { return static_cast<DataType>(val) / DataType{255}; });
}

template <typename DataType>
GLuint Image<DataType>::LoadTexture() {
    // This is translated from if(mData==NULL) but couldn't we just resize here?
    if (mData.empty()) return 0;

    // OpenGL can't handle 2 component images
    GLuint components = (mComponents != 2 ? mComponents : 3);

    // We load the texture as a float image
    GLuint imageSize = mWidth * mHeight * components;
    std::vector<GLubyte> data(imageSize);

    // Copy member storage buffer to byte array
    for (unsigned int y = 0; y < mHeight; y++)
        for (unsigned int x = 0; x < mWidth; x++)
            for (unsigned int c = 0; c < mComponents; c++)
                data[(components * (x + mWidth * y)) + c] = (*this)(x, y, c);

    GLuint texID;

    if (glIsTexture(mTexID) == GL_FALSE) glGenTextures(1, &mTexID);

    glBindTexture(GL_TEXTURE_RECTANGLE_ARB, mTexID);
    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
    glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameterf(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameterf(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexImage2D(GL_TEXTURE_RECTANGLE_ARB, 0, components, mWidth, mHeight, 0,
                 (components == 1 ? GL_LUMINANCE : (components == 3 ? GL_RGB : GL_RGBA)), GL_FLOAT,
                 data.data());

    fprintf(stderr, "Image uploaded to texture memory [ID: %i]\n", mTexID);

    return mTexID;
}

template <typename DataType>
DataType Image<DataType>::Max(int component) {
    DataType m = (*this)(0, 0, component);
    for (int y = 0; y < mHeight; y++)
        for (int x = 0; x < mWidth; x++)
            if ((*this)(x, y, component) > m) m = (*this)(x, y, component);

    return m;
}

template <typename DataType>
DataType Image<DataType>::Min(int component) {
    DataType m = (*this)(0, 0, component);
    for (int y = 0; y < mHeight; y++)
        for (int x = 0; x < mWidth; x++)
            if ((*this)(x, y, component) < m) m = (*this)(x, y, component);

    return m;
}

template <typename DataType>
Image<DataType> &Image<DataType>::operator=(const Image &img) {
    if (this == &img) return *this;

    mWidth = img.mWidth;
    mHeight = img.mHeight;
    mComponents = img.mComponents;

    mData = img.mData;

    return *this;
}
