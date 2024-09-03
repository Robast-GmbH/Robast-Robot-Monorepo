
class Validators {
  static String? mailValidator(String? value) {
    if (value == null || value.isEmpty) {
      return 'Bitte geben Sie ihre E-Mail-Adresse ein';
    }

    // Basic email pattern
    final RegExp emailRegex = RegExp(
      r'^[^@\s]+@[^@\s]+\.[^@\s]+$',
    );

    if (!emailRegex.hasMatch(value)) {
      return 'Bitte geben Sie eine gültige E-Mail-Adresse ein';
    }

    return null;
  }

  static String? nameValidator(String? value) {
    // Check if the input is null or empty
    if (value == null || value.isEmpty) {
      return 'Bitte geben Sie ihren Namen ein';
    }

    // Check if the name is too short or too long
    if (value.length < 2) {
      return 'Namen müssen mindestens 2 Zeichen lang sein';
    }
    if (value.length > 50) {
      return 'Namen dürfen höchstens 50 Zeichen lang sein';
    }

    // Check if the name contains only alphabetic characters, spaces, or hyphens
    final RegExp nameRegex = RegExp(r'^[a-zA-Z\s\-]+$');
    if (!nameRegex.hasMatch(value)) {
      return 'Namen dürfen nur Buchstaben, Leerzeichen und Bindestriche enthalten';
    }

    return null; // Return null if the input is valid
  }

  static String? passwordValidator(String? value) {
    if (value == null || value.isEmpty) {
      return 'Bitte geben Sie Ihr Passwort ein';
    }
    if (value.length < 8) {
      return 'Das Passwort muss mindestens 8 Zeichen lang sein';
    }
    if (!value.contains(RegExp(r'[A-Z]'))) {
      return 'Das Passwort muss mindestens einen Großbuchstaben enthalten';
    }
    if (!value.contains(RegExp(r'[a-z]'))) {
      return 'Das Passwort muss mindestens einen Kleinbuchstaben enthalten';
    }
    if (!value.contains(RegExp(r'[0-9]'))) {
      return 'Das Passwort muss mindestens eine Ziffer enthalten';
    }
    if (!value.contains(RegExp(r'[?!@#\$&*~]'))) {
      return 'Das Passwort muss mindestens ein Sonderzeichen enthalten';
    }
    return null;
  }

  static String? userGroupsValidator(String? value) {
    final regExp = RegExp(r'^([a-zA-ZäöüßÄÖÜ]+,)*[a-zA-ZäöüßÄÖÜ]+$');
    if (value == null || value.isEmpty) {
      return 'Bitte geben Sie mindestens eine Nutzergruppe an';
    } else if (!regExp.hasMatch(value)) {
      return 'Die Eingabe muss eine durch Kommas getrennte Liste von Wörtern sein';
    } else {
      final availableGroups = ['PATIENT', 'STAFF', 'ADMIN'];
      for (final group in value.split(',')) {
        if (!availableGroups.contains(group)) {
          return 'Bitte geben Sie nur gültige Nutzergruppen an';
        }
      }
      return null;
    }
  }
}
