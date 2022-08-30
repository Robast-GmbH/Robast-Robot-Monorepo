import { ClientInnerWidth, ScaleImage } from "react-scale-image"
 
const FancyComponent = () => (
  <ClientInnerWidth>
    {({ width }) => (
      <>
        <ScaleImg url="./image/Logo.png" clientWidth={width} aspect={0.5} />
        <ScaleImg url="/image/Logo.png" clientWidth={width} aspect={0.75} />
      </>
    )}
  </ClientInnerWidth>
)