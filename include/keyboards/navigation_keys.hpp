#ifndef NAVIGATION_KEYBOARDS_NAVIGATION_KEYS_HPP_
#define NAVIGATION_KEYBOARDS_NAVIGATION_KEYS_HPP_

namespace navigation::keyboards
{

int keyAscii(int key);
bool isEscKey(int key);
bool isEnterKey(int key);
bool isBackspaceKey(int key);
bool isUpKey(int key);
bool isDownKey(int key);
bool isLeftKey(int key);
bool isRightKey(int key);
bool isArrowKey(int key);

}  // namespace navigation::keyboards

#endif  // NAVIGATION_KEYBOARDS_NAVIGATION_KEYS_HPP_
