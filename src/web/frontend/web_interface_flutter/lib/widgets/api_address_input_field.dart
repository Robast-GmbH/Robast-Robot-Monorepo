import 'package:flutter/material.dart';
import 'package:web_interface_flutter/services/api_service.dart';

class APIAddressInputField extends StatefulWidget {
  const APIAddressInputField({super.key});

  @override
  State<APIAddressInputField> createState() => _APIAddressInputFieldState();
}

class _APIAddressInputFieldState extends State<APIAddressInputField> {
  final controller = TextEditingController(text: "${APIService.baseURL}:${APIService.port}");

  void showResultDialog(bool isResponding) {
    showDialog(
      context: context,
      builder: (context) => AlertDialog(
        title: const Text("Verbindungstest"),
        content: Text("Der Test war ${isResponding ? "" : "nicht "}erfolgreich."),
        actions: [
          TextButton(
            onPressed: () {
              Navigator.pop(context);
            },
            child: const Text("Okay"),
          ),
        ],
      ),
    );
  }

  @override
  Widget build(BuildContext context) {
    return Center(
      child: Row(
        mainAxisAlignment: MainAxisAlignment.center,
        children: [
          const Text("API Adresse:"),
          const SizedBox(
            width: 16,
          ),
          SizedBox(
              width: 256,
              child: TextField(
                controller: controller,
              )),
          const SizedBox(
            width: 16,
          ),
          TextButton.icon(
            icon: const Icon(Icons.refresh),
            label: const Text("Verbindung testen"),
            onPressed: () async {
              final isResponding = await APIService.testConnection(controller.text);
              showResultDialog(isResponding);
            },
          ),
          const SizedBox(
            width: 16,
          ),
          TextButton.icon(
            label: const Text("Übernehmen"),
            onPressed: () {
              final addressParts = controller.text.split(":");
              if (addressParts.length < 3) return;
              final baseURL = "${addressParts[0]}:${addressParts[1]}";
              final port = int.tryParse(addressParts[2]);
              if (port != null) {
                APIService.baseURL = baseURL;
                APIService.port = port;
              }
            },
            icon: const Icon(Icons.check),
          )
        ],
      ),
    );
  }
}
