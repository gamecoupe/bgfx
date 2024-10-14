#ifndef TRYBGFX_DEBUGDRAW_H_INPUT_WIN_GUARD
#define TRYBGFX_DEBUGDRAW_H_INPUT_WIN_GUARD

#include "input.h"
#include <Windows.h>
//
//class KeyboadInputFormat :public InputButtonFormatBase<char>
//{
//public:
//	KeyboadInputFormat()
//	{
//		// キーを登録
//		addButton("A", 'A');
//		addButton("B", 'B');
//		addButton("C", 'C');
//		addButton("D", 'D');
//		addButton("E", 'E');
//		addButton("F", 'F');
//		addButton("G", 'G');
//		addButton("H", 'H');
//		addButton("I", 'I');
//		addButton("J", 'J');
//		addButton("K", 'K');
//		addButton("L", 'L');
//		addButton("M", 'M');
//		addButton("N", 'N');
//		addButton("O", 'O');
//		addButton("P", 'P');
//		addButton("Q", 'Q');
//		addButton("R", 'R');
//		addButton("S", 'S');
//		addButton("T", 'T');
//		addButton("U", 'U');
//		addButton("V", 'V');
//		addButton("W", 'W');
//		addButton("X", 'X');
//		addButton("Y", 'Y');
//		addButton("Z", 'Z');
//
//		addButton("Up", VK_UP);
//		addButton("Down", VK_DOWN);
//		addButton("Left", VK_LEFT);
//		addButton("Right", VK_RIGHT);
//
//		addButton("Space", VK_SPACE);
//		addButton("Return", VK_RETURN);
//
//		addButton("RClick", VK_RBUTTON);
//		addButton("LClick", VK_LBUTTON);
//	}
//
//	const bool checkButtonState(char a_keyCode) override
//	{
//		return GetAsyncKeyState(a_keyCode) & 0x8000;
//	}
//};
//
//
//class MouseInputFormat :public InputAxisFormatBase<char>
//{
//public:
//	MouseInputFormat()
//	{
//		POINT cursorPos;
//		GetCursorPos(&cursorPos);
//		m_fixCursorPos = cursorPos;
//
//		//addKey("Mouse");
//		addAxis("Mouse", 0);
//	}
//	const AxisStatus checkAxisState(char) override
//	{
//		POINT cursorPos;
//		GetCursorPos(&cursorPos);
//		AxisStatus result = { static_cast<float>(cursorPos.x) - m_fixCursorPos.x,
//			static_cast<float>(cursorPos.y) - m_fixCursorPos.y };
//		m_fixCursorPos = cursorPos;
//		return result;
//	}
//
//private:
//	POINT m_fixCursorPos = {};
//};
//
//
//class WindowsInputDevice :public InputDeviceBase<char>
//{
//public:
//	WindowsInputDevice()
//	{
//		// 入力フォーマットを登録
//		addInputButtonFormat<KeyboadInputFormat>("keyboad");
//		addInputAxisFormat<MouseInputFormat>("mouse");
//	}
//};


#endif
