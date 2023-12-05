import 'package:flutter/material.dart';
import 'package:web_interface_flutter/constants/app_colors.dart';

class DetailsInputField extends StatelessWidget {
  const DetailsInputField({
    super.key,
    required this.controller,
    this.hintText,
    this.autocompleteValues,
  });

  final TextEditingController controller;
  final String? hintText;
  final List<String>? autocompleteValues;

  @override
  Widget build(BuildContext context) {
    FocusNode? focusNode;
    return GestureDetector(
      onTap: () {
        focusNode?.requestFocus();
      },
      child: Container(
        decoration: BoxDecoration(
          color: AppColors.grey,
          borderRadius: BorderRadius.circular(8),
        ),
        padding: const EdgeInsets.all(24),
        child: autocompleteValues == null
            ? TextFormField(
                style: const TextStyle(fontSize: 24),
                focusNode: focusNode,
                decoration: InputDecoration.collapsed(hintText: hintText),
                controller: controller,
              )
            : Autocomplete<String>(
                optionsBuilder: (textEditingValue) {
                  return autocompleteValues!.where(
                    (element) => element.contains(textEditingValue.text),
                  );
                },
                onSelected: (value) {},
                fieldViewBuilder: (context, textEditingController, focusNode2, onFieldSubmitted) {
                  focusNode = focusNode2;
                  return TextFormField(
                    style: const TextStyle(fontSize: 24),
                    focusNode: focusNode2,
                    decoration: InputDecoration.collapsed(hintText: hintText),
                    controller: textEditingController,
                  );
                },
              ),
      ),
    );
  }
}
