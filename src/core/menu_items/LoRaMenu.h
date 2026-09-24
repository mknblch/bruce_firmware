#ifndef __LORA_MENU_H__
#define __LORA_MENU_H__

#if !defined(LITE_VERSION)
#include <MenuItemInterface.h>

class LoRaMenu : public MenuItemInterface {
public:
    LoRaMenu() : MenuItemInterface("LoRa") {}

    void optionsMenu(void);
    void drawIcon(float scale) override;
    bool hasTheme() override { return bruceConfig.theme.lora; }
    const String& themePath() override { return bruceConfig.theme.paths.lora; }
};

#endif // !LITE_VERSION
#endif // __LORA_MENU_H__
