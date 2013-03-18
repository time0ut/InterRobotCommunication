#! /bin/sh
# Sets work environment (libpcre...)

# Which PCRE version to use
PCREVER=8.32

# Create directories
# To put compiled stuff
LOCAL=local
GSTIX=gumstix
PREFIXLOCAL=$(pwd)/$LOCAL
PREFIXGSTIX=$(pwd)/$GSTIX
export PREFIX

mkdir -p $LOCAL/lib/ivy $LOCAL/include/Ivy $GSTIX/lib/ivy $GSTIX/include/Ivy

# ---- PCRE
# Download tarball
echo "*** Downloading PCRE sources"
wget "http://downloads.sourceforge.net/project/pcre/pcre/$PCREVER/pcre-$PCREVER.tar.gz"
echo "*** Done"
# Extract
echo "*** Extracting PCRE sources"
tar xvf pcre-$PCREVER.tar.gz
echo "*** Done"
# Build and install
echo "*** Building PCRE"
cd pcre-$PCREVER
chmod +x ./configure
echo "**** For local"
./configure --prefix=$PREFIXLOCAL
make
make install
make clean
echo "**** Done"
echo "**** For gumstix"
./configure --prefix=$PREFIXGSTIX --host=arm-linux
make
make install
make clean
echo "**** Done"
echo "*** Done"
cd ..
# Clean folder
rm -r pcre-$PCREVER*

# ---- Ivy

# Checkout
echo "*** Retrieving Ivy sources"
svn co http://svn.tls.cena.fr/svn/ivy/ivy-c/trunk Ivy/ivy-c/

echo "*** Building Ivy"
# Compile for local
echo "**** For local"
cd Ivy/ivy-c/src
export PERHAPS64=''
export PREFIX=$PREFIXLOCAL
export PCREPREFIX=$PREFIX
export PCREINC=-I$PCREPREFIX/include
export PCRELIB=-I$PCREPREFIX/include
export PCREOBJ=$PCREPREFIX/lib/libpcre.a
export CPPFLAGS="-MMD -I$PREFIX/include/"
make clean
make -e libivy.a
cp -uv libivy.a $PREFIX/lib/ivy/
cp -uv *.h $PREFIX/include/Ivy/
make clean
cd ../../..
echo "**** Done"

# Compile for gumstix
echo "**** For gumstix"
cd Ivy/ivy-c/src
export PERHAPS64=''
export PREFIX=$PREFIXGSTIX
export FPIC=-fPIC
export PCREPREFIX=$PREFIX
export PCREINC=-I$PCREPREFIX/include
export PCRELIB=-I$PCREPREFIX/include
export PCREOBJ=$PCREPREFIX/lib/libpcre.a
export CC='arm-linux-gcc'
export CPP='arm-linux-g++'
make -e libivy.a
cp -uv libivy.a $PREFIX/lib/ivy/
cp -uv *.h $PREFIX/include/Ivy/
make clean
cd ../../..
echo "**** Done"
echo "*** Done"

cd ../..

