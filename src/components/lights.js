import { DirectionalLight } from '@babylonjs/core/Lights/directionalLight'
import { HemisphericLight } from '@babylonjs/core/Lights/hemisphericLight'
import { ShadowGenerator } from '@babylonjs/core/Lights/Shadows/shadowGenerator'
import { Vector3 } from '@babylonjs/core/Maths/math.vector'
import '@babylonjs/core/Lights/Shadows/shadowGeneratorSceneComponent'

const defaultOptions = {
  enableShadows: true
}

function createLights(options = defaultOptions) {
  const { enableShadows, scene } = options
  const d_light = new DirectionalLight("DirectionalLight", new Vector3(-0.3, -1, 0.4), scene)
  d_light.position = new Vector3(0,30,0)
  d_light.intensity = .3
  
  const h_light = new HemisphericLight("HemisphericLight", new Vector3(1, 1, 0), scene)
  h_light.intensity = .7
  
  if(enableShadows){
    d_light.shadowMinZ = 1
    d_light.shadowMaxZ = 40
		// d_light.autoCalcShadowZBounds = true
    d_light.shadowGenerator = new ShadowGenerator(1024, d_light);
    d_light.shadowGenerator.useCloseExponentialShadowMap = true; // best
    d_light.shadowGenerator.darkness = .7;
  }

  return {directional: d_light, hemispheric: h_light}
}

export { createLights }