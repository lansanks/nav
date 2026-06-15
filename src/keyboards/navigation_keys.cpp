#include "keyboards/navigation_keys.hpp"

namespace navigation::keyboards
{

int keyAscii(int key)
{
  if (key == -1) {
    return -1;
  }
  return key & 0xff;
}

bool isEscKey(int key)
{
  return keyAscii(key) == 27;
}

bool isEnterKey(int key)
{
  const int ascii = keyAscii(key);
  return ascii == 10 || ascii == 13;
}

bool isBackspaceKey(int key)
{
  const int ascii = keyAscii(key);
  return ascii == 8 || ascii == 127;
}

bool isUpKey(int key)
{
  return key == 65362 || key == 2490368 || key == 82 || key == 1113938;
}

bool isDownKey(int key)
{
  return key == 65364 || key == 2621440 || key == 84 || key == 1113940;
}

bool isLeftKey(int key)
{
  return key == 65361 || key == 2424832;
}

bool isRightKey(int key)
{
  return key == 65363 || key == 2555904;
}

bool isArrowKey(int key)
{
  return isUpKey(key) || isDownKey(key) || isLeftKey(key) || isRightKey(key);
}

}  // namespace navigation::keyboards
