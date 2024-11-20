import 'package:flutter/material.dart';

class CustomTextField extends StatelessWidget {
  const CustomTextField({
    required this.label,
    this.prefixIcon,
    this.controller,
    this.obsucureText = false,
    this.validator,
    super.key,
  });

  final TextEditingController? controller;
  final bool obsucureText;
  final String label;
  final Icon? prefixIcon;
  final String? Function(String?)? validator;

  @override
  Widget build(BuildContext context) {
    return TextFormField(
      controller: controller,
      obscureText: obsucureText,
      style: const TextStyle(color: Colors.white),
      decoration: InputDecoration(label: Text(label), prefixIcon: prefixIcon, border: const OutlineInputBorder()),
      validator: validator,
    );
  }
}
