from kivymd.app import MDApp
from kivy.lang import Builder

from Controller.app_controller import AppController


class CocktailApp(MDApp):
    def build(self):
        self.title = "Mixmate – die automatische Schankanlage"
        self.theme_cls.theme_style = "Dark"
        self.theme_cls.primary_palette = "Teal"
        self.theme_cls.accent_palette = "Cyan"

        # Lade KV-Dateien
        Builder.load_file("View/screens.kv")

        # Controller initialisieren
        self.controller = AppController(app=self)
        return self.controller.sm


if __name__ == "__main__":
    CocktailApp().run()
