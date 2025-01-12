#ifndef TRYBGFX_DEBUGDRAW_H_INPUT_GUARD
#define TRYBGFX_DEBUGDRAW_H_INPUT_GUARD

#include "bx/bx.h"

#include "tinystl/unordered_map.h"
#include "tinystl/unordered_set.h"
#include "tinystl/string.h"

#include <memory>
#include <Windows.h>


class InputTypeManager
{
public:
	static inline const uint32_t generateID()
	{
		return ++m_nextInputTypeID;
	}
private:
	static uint32_t m_nextInputTypeID;
};

//uint32_t InputTypeManager::m_nextInputTypeID = 0;



enum KeyStatus
{
	Free = 1 << 0,
	Press = 1 << 1,
	Hold = 1 << 2,
	Release = 1 << 3
};

template<typename KeyCodeType>
class InputButtonFormatBase
{
public:
	void addButton(tinystl::string _buttonName, KeyCodeType _keyCode)
	{
		m_buttonNameToKeyCode.insert(tinystl::make_pair(_buttonName, _keyCode));
	}

	void removeButton(tinystl::string _buttonName)
	{
		auto it = m_buttonNameToKeyCode.find(_buttonName);
		if (it != m_buttonNameToKeyCode.end())
		{
			m_buttonNameToKeyCode.erase(it);
		}
	}

	virtual const bool checkButtonState(KeyCodeType _keyCode) = 0;

	static inline const uint32_t getTypeID()
	{
		if (!m_buttonFormatTypeID)
		{
			InputButtonFormatBase<KeyCodeType>::m_buttonFormatTypeID = InputTypeManager::generateID();
		}
		return InputButtonFormatBase<KeyCodeType>::m_buttonFormatTypeID;
	}

	inline const tinystl::unordered_map<tinystl::string, KeyCodeType>& getButtonNameToKeyCode()
	{
		return m_buttonNameToKeyCode;
	}


private:
	tinystl::unordered_map<tinystl::string, KeyCodeType> m_buttonNameToKeyCode;
	static uint32_t m_buttonFormatTypeID;
};

template<typename KeyCodeType>
uint32_t InputButtonFormatBase<KeyCodeType>::m_buttonFormatTypeID = 0;



struct AxisStatus
{
	AxisStatus() = default;
	AxisStatus(const float _width, const float _height)
		:m_width(_width)
		, m_height(_height)
	{}

	AxisStatus operator+(const AxisStatus& _value)
	{
		return AxisStatus(m_width + _value.m_width, m_height + _value.m_height);
	}
	AxisStatus operator+=(const AxisStatus& _value)
	{
		m_width += _value.m_width;
		m_height += _value.m_height;
		return *this;
	}

	float m_width;
	float m_height;
};

template<typename KeyCodeType>
class InputAxisFormatBase
{
public:
	inline void addAxis(tinystl::string _axisName, KeyCodeType _keyCode)
	{
		m_axisNameToKeyCode.insert(tinystl::make_pair(_axisName, _keyCode));
	}

	inline void removeAxis(tinystl::string _axisName)
	{
		auto it = m_axisNameToKeyCode.find(_axisName);
		if (it != m_axisNameToKeyCode.end())
		{
			m_axisNameToKeyCode.erase(it);
		}
	}

	virtual const AxisStatus checkAxisState(KeyCodeType _keyCode) = 0;

	static inline const uint32_t getTypeID()
	{
		if (!m_axisFormatTypeID)
		{
			InputAxisFormatBase<KeyCodeType>::m_axisFormatTypeID = InputTypeManager::generateID();
		}
		return InputAxisFormatBase<KeyCodeType>::m_axisFormatTypeID;
	}

	inline const tinystl::unordered_map<tinystl::string, KeyCodeType>& getAxisNameToKeyCode()
	{
		return m_axisNameToKeyCode;
	}

private:
	tinystl::unordered_map<tinystl::string, KeyCodeType> m_axisNameToKeyCode;
	static uint32_t m_axisFormatTypeID;
};

template<typename KeyCodeType>
uint32_t InputAxisFormatBase<KeyCodeType>::m_axisFormatTypeID = 0;



class IInputDeviceBase
{
public:
	virtual void update() = 0;

	virtual short getButtonState(tinystl::string _buttonName) = 0;
	virtual const bool isButtonFree(tinystl::string _buttonName) = 0;
	virtual const bool isButtonPress(tinystl::string _buttonName) = 0;
	virtual const bool isButtonHold(tinystl::string _buttonName) = 0;
	virtual const bool isButtonRelease(tinystl::string _buttonName) = 0;
	virtual const tinystl::unordered_map<tinystl::string, short>& getAllButtonResult() = 0;

	virtual const AxisStatus getAxisState(tinystl::string _axisName) = 0;
	virtual const tinystl::unordered_map<tinystl::string, AxisStatus>& getAllAxisResult() = 0;
};

template<typename KeyCodeType>
class InputDeviceBase :public IInputDeviceBase
{
public:

	virtual void update()override final
	{
		// Button
		for (auto&& format : m_nameToButtonFormat)
		{
			for (auto&& button : format.second->getButtonNameToKeyCode())
			{
				short result = m_buttonToResult[button.first];
				if (format.second->checkButtonState(button.second))
				{
					if (result & KeyStatus::Free)
					{
						result |= KeyStatus::Press;
					}
					else
					{
						result &= ~KeyStatus::Press;
					}
					result &= ~KeyStatus::Free;
					result |= KeyStatus::Hold;
				}
				else
				{
					if (result & KeyStatus::Hold)
					{
						result |= KeyStatus::Release;
					}
					else
					{
						result &= ~KeyStatus::Release;
					}
					result &= ~KeyStatus::Hold;
					result |= KeyStatus::Free;
				}
				m_buttonToResult[button.first] = result;
			}
		}

		// Axis
		for (auto&& format : m_nameToAxisFormat)
		{
			for (auto&& axis : format.second->getAxisNameToKeyCode())
			{
				m_axisToResult[axis.first] = format.second->checkAxisState(axis.second);
			}
		}
	}

	// Button
	short getButtonState(tinystl::string _buttonName)override final
	{
		auto itr = m_buttonToResult.find(_buttonName);
		if (itr != m_buttonToResult.end())
		{
			return itr->second;
		}
		return 0;
	}

	const bool isButtonFree(tinystl::string _buttonName)override final
	{
		auto itr = m_buttonToResult.find(_buttonName);
		if (itr != m_buttonToResult.end())
		{
			return itr->second & KeyStatus::Free;
		}
		return false;
	}

	const bool isButtonPress(tinystl::string _buttonName)override final
	{
		auto itr = m_buttonToResult.find(_buttonName);
		if (itr != m_buttonToResult.end())
		{
			return itr->second & KeyStatus::Press;
		}
		return false;
	}

	const bool isButtonHold(tinystl::string _buttonName)override final
	{
		auto itr = m_buttonToResult.find(_buttonName);
		if (itr != m_buttonToResult.end())
		{
			return itr->second & KeyStatus::Hold;
		}
		return false;
	}

	const bool isButtonRelease(tinystl::string _buttonName)override final
	{
		auto itr = m_buttonToResult.find(_buttonName);
		if (itr != m_buttonToResult.end())
		{
			return itr->second & KeyStatus::Release;
		}
		return false;
	}

	const tinystl::unordered_map<tinystl::string, short>& getAllButtonResult()override final
	{
		return m_buttonToResult;
	}

	// Axis
	const AxisStatus getAxisState(tinystl::string _axisName)override final
	{
		auto itr = m_axisToResult.find(_axisName);
		if (itr != m_axisToResult.end())
		{
			return itr->second;
		}
		return AxisStatus();
	}

	const tinystl::unordered_map<tinystl::string, AxisStatus>& getAllAxisResult()override final
	{
		return m_axisToResult;
	}


	template<typename FormatType>
	void addInputButtonFormat(tinystl::string _formatName)
	{
		uint32_t type = FormatType::getTypeID();
		m_formatTypeToName[type] = _formatName;
		m_nameToFormatType[_formatName] = type;
		m_nameToButtonFormat[_formatName] = std::make_shared<FormatType>();
	}

	template<typename FormatType>
	void addInputAxisFormat(tinystl::string _formatName)
	{
		uint32_t type = FormatType::getTypeID();
		m_formatTypeToName[type] = _formatName;
		m_nameToFormatType[_formatName] = type;
		m_nameToAxisFormat[_formatName] = std::make_shared<FormatType>();
	}

	static inline const uint32_t getTypeID()
	{
		if (!InputDeviceBase<KeyCodeType>::m_deviceTypeID)
		{
			InputDeviceBase<KeyCodeType>::m_deviceTypeID = InputTypeManager::generateID();
		}
		return InputDeviceBase<KeyCodeType>::m_deviceTypeID;
	}

protected:
	tinystl::unordered_map<tinystl::string, std::shared_ptr<InputButtonFormatBase<KeyCodeType>>> m_nameToButtonFormat;
	tinystl::unordered_map<tinystl::string, std::shared_ptr<InputAxisFormatBase<KeyCodeType>>> m_nameToAxisFormat;

	tinystl::unordered_map<uint32_t, tinystl::string> m_formatTypeToName;
	tinystl::unordered_map<tinystl::string, uint32_t> m_nameToFormatType;

	tinystl::unordered_map<tinystl::string, short> m_buttonToResult;
	tinystl::unordered_map<tinystl::string, AxisStatus> m_axisToResult;

	static uint32_t m_deviceTypeID;

};
template<typename KeyCodeType>
uint32_t InputDeviceBase<KeyCodeType>::m_deviceTypeID = 0;



class InputDeviceCentor
{
public:
	template<typename DeviceType>
	inline std::shared_ptr<DeviceType> addDevice(tinystl::string _deviceName)
	{
		std::shared_ptr<DeviceType> spNewDevice = std::make_shared<DeviceType>();
		m_nameToDevice[_deviceName] = spNewDevice;

		m_deviceTypeToName[DeviceType::getTypeID()] = _deviceName;

		return spNewDevice;
	}

	template<typename DeviceType>
	inline std::shared_ptr<DeviceType> getDevice()
	{
		uint32_t type = DeviceType::GetTypeID();
		auto itr = m_deviceTypeToName.find(type);
		if (itr != m_deviceTypeToName.end())
		{
			return std::static_pointer_cast<DeviceType>(m_nameToDevice[itr->second]);
		}
		return nullptr;
	}

	inline std::shared_ptr<IInputDeviceBase> getDevice(tinystl::string _name)
	{
		auto itr = m_nameToDevice.find(_name);
		if (itr != m_nameToDevice.end())
		{
			return itr->second;
		}
		return nullptr;
	}

	inline const tinystl::unordered_map<tinystl::string, std::shared_ptr<IInputDeviceBase>>& getAllDevice()
	{
		return m_nameToDevice;
	}

	void update()
	{
		for (auto&& device : m_nameToDevice)
		{
			device.second->update();
		}
	}


private:
	tinystl::unordered_map<tinystl::string, std::shared_ptr<IInputDeviceBase>> m_nameToDevice;
	tinystl::unordered_map<uint32_t, tinystl::string> m_deviceTypeToName;
};



class InputCollector
{
public:
	inline const bool checkActive()
	{
		return m_active;
	}

	inline void setActive(const bool _bActive)
	{
		m_active = _bActive;
	}

	inline tinystl::string getName()
	{
		return m_name;
	}

	inline void setName(tinystl::string _name)
	{
		m_name = _name;
	}

	// Button
	inline void addButtonWork(tinystl::string _work)
	{
		m_workToButtons[_work];
	}

	inline void addButtonToButtonWork(tinystl::string _work, tinystl::string _device, tinystl::string _button)
	{
		m_workToButtons[_work][_device].insert(_button);
	}

	void removeButtonWork(tinystl::string _work)
	{
		auto it = m_workToButtons.find(_work);
		if (it != m_workToButtons.end())
		{
			m_workToButtons.erase(it);
		}
	}

	inline void removeButtonFromButtonWork(tinystl::string _work, tinystl::string _device, tinystl::string _button)
	{
		m_workToButtons[_work][_device].erase(_button);
	}

	inline const tinystl::unordered_map<tinystl::string, tinystl::unordered_map<tinystl::string, tinystl::unordered_set<tinystl::string>>>& getAllButtonWork()
	{
		return m_workToButtons;
	}

	// Axis
	inline void addAxisWork(tinystl::string _work)
	{
		m_workToAxises[_work];
	}

	void addAxisToAxisWork(tinystl::string _work, tinystl::string _device, tinystl::string _axis)
	{
		m_workToAxises[_work][_device].insert(_axis);
	}

	inline void removeAxisWork(tinystl::string _work)
	{
		auto it = m_workToAxises.find(_work);
		if (it != m_workToAxises.end())
		{
			m_workToAxises.erase(it);
		}
	}

	inline void removeAxisFromAxisWork(tinystl::string _work, tinystl::string _device, tinystl::string _axis)
	{
		m_workToAxises[_work][_device].erase(_axis);
	}


	inline const tinystl::unordered_map<tinystl::string, tinystl::unordered_map<tinystl::string, tinystl::unordered_set<tinystl::string>>>& getAllAxisWork()
	{
		return m_workToAxises;
	}

private:
	bool m_active = false;
	tinystl::string m_name;

	tinystl::unordered_map<tinystl::string, tinystl::unordered_map<tinystl::string, tinystl::unordered_set<tinystl::string>>> m_workToButtons;
	tinystl::unordered_map<tinystl::string, tinystl::unordered_map<tinystl::string, tinystl::unordered_set<tinystl::string>>> m_workToAxises;

};



class Input
{
public:
	// Device
	template<typename DeviceType>
	inline void addDevice(tinystl::string _deviceName)
	{
		m_deviceCentor.addDevice<DeviceType>(_deviceName);
	}

	template<typename DeviceType>
	inline std::shared_ptr<DeviceType> getDevice()
	{
		return m_deviceCentor.getDevice<DeviceType>();
	}

	std::shared_ptr<IInputDeviceBase> getDevice(tinystl::string _deviceName)
	{
		return m_deviceCentor.getDevice(_deviceName);
	}

	inline const tinystl::unordered_map<tinystl::string, std::shared_ptr<IInputDeviceBase>>& getAllDevice()
	{
		return m_deviceCentor.getAllDevice();
	}

	// Collector
	inline std::shared_ptr<InputCollector> addCollector(tinystl::string _collectorName)
	{
		std::shared_ptr<InputCollector> spCollector = std::make_shared<InputCollector>();
		m_nameToCollector[_collectorName] = spCollector;
		spCollector->setName(_collectorName);
		spCollector->setActive(true);
		return spCollector;
	}

	inline std::shared_ptr<InputCollector>getCollector(tinystl::string _collectorName)
	{
		return m_nameToCollector[_collectorName];
	}

	inline tinystl::unordered_map<tinystl::string, std::shared_ptr<InputCollector>>& getAllCollector()
	{
		return m_nameToCollector;
	}

	// Work
	inline const short getButtonWorkState(tinystl::string _work)
	{
		return m_workToButtonResult[_work];
	}

	inline const bool isButtonWorkFree(tinystl::string _work)
	{
		return m_workToButtonResult[_work] & KeyStatus::Free;
	}

	inline const bool isButtonWorkPress(tinystl::string _work)
	{
		return m_workToButtonResult[_work] & KeyStatus::Press;
	}

	inline const bool isButtonWorkHold(tinystl::string _work)
	{
		return m_workToButtonResult[_work] & KeyStatus::Hold;
	}

	inline const bool isButtonWorkRelease(tinystl::string _work)
	{
		return m_workToButtonResult[_work] & KeyStatus::Release;
	}

	inline const AxisStatus getAxisStatus(tinystl::string _work)
	{
		AxisStatus res = m_workToAxisResult[_work];
		return res;
	}


	void update()
	{
		m_deviceCentor.update();

		m_workToButtonResult.clear();
		m_workToAxisResult.clear();

		for (auto&& collector : m_nameToCollector)
		{
			if (!collector.second->checkActive())
			{
				continue;
			}

			for (auto&& work : collector.second->getAllButtonWork())
			{
				for (auto&& deviceToButtons : work.second)
				{
					for (auto&& button : deviceToButtons.second)
					{
						m_workToButtonResult[work.first] |=
							m_deviceCentor.getDevice(deviceToButtons.first)->getButtonState(button);
					}
				}
			}

			for (auto&& work : collector.second->getAllAxisWork())
			{
				for (auto&& deviceToAxises : work.second)
				{
					for (auto&& axis : deviceToAxises.second)
					{
						m_workToAxisResult[work.first] +=
							m_deviceCentor.getDevice(deviceToAxises.first)->getAxisState(axis);
					}
				}
			}
		}
	}

private:
	InputDeviceCentor m_deviceCentor;

	tinystl::unordered_map<tinystl::string, std::shared_ptr<InputCollector>> m_nameToCollector;

	tinystl::unordered_map<tinystl::string, short> m_workToButtonResult;
	tinystl::unordered_map<tinystl::string, AxisStatus> m_workToAxisResult;
};



class KeyboadInputFormat :public InputButtonFormatBase<char>
{
public:
	KeyboadInputFormat()
	{
		addButton("A", 'A');
		addButton("B", 'B');
		addButton("C", 'C');
		addButton("D", 'D');
		addButton("E", 'E');
		addButton("F", 'F');
		addButton("G", 'G');
		addButton("H", 'H');
		addButton("I", 'I');
		addButton("J", 'J');
		addButton("K", 'K');
		addButton("L", 'L');
		addButton("M", 'M');
		addButton("N", 'N');
		addButton("O", 'O');
		addButton("P", 'P');
		addButton("Q", 'Q');
		addButton("R", 'R');
		addButton("S", 'S');
		addButton("T", 'T');
		addButton("U", 'U');
		addButton("V", 'V');
		addButton("W", 'W');
		addButton("X", 'X');
		addButton("Y", 'Y');
		addButton("Z", 'Z');

		addButton("Up", VK_UP);
		addButton("Down", VK_DOWN);
		addButton("Left", VK_LEFT);
		addButton("Right", VK_RIGHT);

		addButton("Space", VK_SPACE);
		addButton("Return", VK_RETURN);

		addButton("RClick", VK_RBUTTON);
		addButton("LClick", VK_LBUTTON);
	}

	const bool checkButtonState(char a_keyCode) override
	{
		return GetAsyncKeyState(a_keyCode) & 0x8000;
	}
};


class MouseInputFormat :public InputAxisFormatBase<char>
{
public:
	MouseInputFormat()
	{
		POINT cursorPos;
		GetCursorPos(&cursorPos);
		m_fixCursorPos = cursorPos;

		//addKey("Mouse");
		addAxis("Cursor", 0);
	}
	const AxisStatus checkAxisState(char) override
	{
		POINT cursorPos;
		GetCursorPos(&cursorPos);
		AxisStatus result = { static_cast<float>(cursorPos.x) - m_fixCursorPos.x,
			static_cast<float>(cursorPos.y) - m_fixCursorPos.y };
		m_fixCursorPos = cursorPos;
		return result;
	}

private:
	POINT m_fixCursorPos = {};
};


class WindowsInputDevice :public InputDeviceBase<char>
{
public:
	WindowsInputDevice()
	{
		// 入力フォーマットを登録
		addInputButtonFormat<KeyboadInputFormat>("keyboad");
		addInputAxisFormat<MouseInputFormat>("mouse");
	}
};



#endif
